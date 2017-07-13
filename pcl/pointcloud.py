'''
Implementation of following files:
    pcl/common/include/pcl/impl/point_types.hpp
    pcl/common/include/pcl/point_cloud.h
    pcl/common/include/pcl/PCLPointField.h
    pcl/common/include/pcl/point_traits.h
    pcl/common/include/pcl/point_types.h
    pcl/common/include/pcl/register_point_struct.h
    pcl/common/include/pcl/point_traits.h

Following files are abandoned:
    pcl/common/include/pcl/impl/cloud_iterator.hpp
    pcl/common/include/pcl/impl/instantiate.hpp
    pcl/common/include/pcl/point_types_conversion.h
    pcl/common/include/pcl/PCLPointCloud2.h
'''
from __future__ import absolute_import

import re
# from copy import copy as cp
import numpy as np
from .quaternion import Quaternion

def _pcl_to_txt(dtype):
    if str.isdigit(str(dtype[0])):
        typen = dtype[1]
        size = dtype[0]
    else:
        typen = dtype[0]
        size = dtype[1]
    count = dtype[2] if len(dtype) == 3 else 1
    return ('' if count == 1 else str(count)) + typen.lower() + str(size)

def _numpy_to_txt(dtype):
    # cast single numpy dtype into txt
    if dtype.subdtype is None:
        return dtype.descr[0][1][1:]
    else:
        shape = dtype.subdtype[1][0] if len(dtype.subdtype[1]) == 1 else dtype.subdtype[1]
        return str(shape) + _numpy_to_txt(dtype.subdtype[0])

def _cast_fields_to_tuples(dtype):
    # Cast point fields into specific tuples that can be used by point cloud
    ftuples = []
    predict = False

    if dtype is None:
        pass
    elif isinstance(dtype, np.dtype):
        ftuples = [(name, _numpy_to_txt(dtype[name])) for name in dtype.names]
    elif isinstance(dtype, (list, tuple)):
        for field in dtype:
            if isinstance(field, str):
                ftuples.append(field)
                predict = True
            elif isinstance(field[1], str) and len(field) == 2:
                if len(field[1]) != len(re.match('(\\d+)?[iufIUF][1248]', field[1]).group()):
                    raise ValueError('Unknown input descriptor')
                ftuples.append((field[0], field[1]))
            else:
                ftuples.append((field[0], _pcl_to_txt(field[1:])))
    # elif isinstance(dtype, dict):
    else:
        raise ValueError('Unknown fields format')

    return ftuples, predict

def _safe_len(array):
    # avoid errors when coded like len(np.array(1))
    if array.ndim == 0:
        # occurs when only one point
        assert array.size == 1
        return array.size
    else:
        return len(array)

class PointCloud:
    '''
    PointCloud represents the base class in PCL for storing collections of 3D points.

    # Operations
    Elements in the ndarray can be accessed by rows and columns (organized point cloud)
    or by indices with [] indexer.
    Fields can also be accessed and edited by field names with [] indexer.
    The indexer returns a **PointCloud object** with the querying data.

    If you want to access raw data of a point, use 'data' property
    '''
    def __init__(self, points=None, fields=None, indices=None, width=0, height=0, copy=True,
                 origin=None, orientation=None):
        '''
        # Parameters
            points : PointCloud or seqence that can be turned into numpy.ndarray
                Points filling the point cloud
            fields : list of tuples or numpy.dtype
                Fields that determine the property of each column of the point data
                The fields can be initialized in following format:
                - PCL tuples (name, size, type, count): e.g. ('normal', 4, 'U', 3),
                    type and size can be swapped
                - PCL tuples (name, size, type): e.g. ('intensity', 1, 'F'),
                    count is by default set to 1
                - tuples/list (name, descriptor) e.g. ('normal': '3u4')
                - numpy.dtype: e.g. dtype(('normal', '<u4', (3,)))
                - name list: ('normal', 'intensity'), type of the fields will be guessed from points

                All kinds of input will be converted into a (field, descriptor) list
            indices : sequence
                Index of the subset to copy
            width : int
                The cloud width
            height : int
                The cloud height
            copy : bool
                Determine if the data from points are copied
            origin : ndarray
                The origin of the sensor
            orientation : quaternion
                The orientation of the sensor
        '''

        if isinstance(points, type(self)): # copy initialization
            self.__points = np.array(points.data, copy=copy)
            self.__fields = list(points.fields) if copy else points.fields
            self.__width = points.width
            self.__height = points.height
            self.__sensor_origin = np.array(points.sensor_origin, copy=copy)
            self.__sensor_orientation = Quaternion(points.sensor_orientation)
        else:
            self.__fields, predict = _cast_fields_to_tuples(fields)
            if points is not None and len(fields) == 1:
                self.__points = np.array(points, fields=self.__fields, copy=copy)
                if self.__points.ndim > 1:
                    raise ValueError("the input points is not an one-dimensional array")
            elif points is not None:
                direct = False # whether converting is not essential
                try: # judging whether its the same type
                    dtype = np.dtype(self.__fields)
                    if dtype == points.dtype:
                        self.__points = np.array(points, copy=copy)
                        self.__fields = fields
                        direct = True
                except:
                    pass

                # dtype directly from ndarray, only multi-fields array is allowed
                if not fields and isinstance(points, np.ndarray) and len(points.dtype) > 0:
                    self.__points = np.array(points, copy=copy)
                    self.__fields = [(name, _numpy_to_txt(self.__points.dtype[name]))
                                     for name in self.__points.dtype.names]
                    direct = True

                if not direct:
                    # converting to tuple and then to ndarray
                    try:
                        points = [tuple(point) for point in points]
                    except TypeError:
                        pass # single value
                    if predict:
                        self.__points = np.rec.fromrecords(points, names=self.__fields)
                        self.__fields = [(name, _numpy_to_txt(self.__points.dtype[name]))
                                         for name in self.__fields]
                    else:
                        self.__points = np.array(points, dtype=self.__fields, copy=copy)
            else:
                if predict:
                    raise TypeError('specifying fields of null cloud with only names')
                if len(self.__fields) > 0:
                    self.__points = np.array([], dtype=self.__fields) # Turn None into null array
                else:
                    self.__points = np.array([])

            self.__width = abs(width)
            self.__height = abs(height)

            # adjust points with width and height automatically
            if width is 0:
                if _safe_len(self.__points) > 0:
                    width = _safe_len(self.__points)
                    height = 1
                elif height > 0:
                    width = height
                    height = 1
            if width > 0:
                if height is 0:
                    height = 1
                if not width*height == _safe_len(self.__points) and _safe_len(self.__points) != 0:
                    raise ValueError("the input width and height doesn't match the count of points")
                self.__width = width
                self.__height = height
                if _safe_len(self.__points) == 0:
                    if len(self.__fields) > 0:
                        self.__points = np.empty(width * height, dtype=self.__fields)
                    else:
                        self.__points = np.empty(width * height)

            self.__sensor_origin = np.zeros(4, dtype=np.int8) if origin is None else origin
            self.__sensor_orientation = Quaternion.identity() if orientation is None \
                                                              else orientation

        # apply indices filter
        if indices is not None:
            self.__points = self.__points[indices]

    def __add__(self, pointcloud):
        copypc = PointCloud(self)
        copypc.append(pointcloud)
        return copypc

    def __array__(self, *_):
        # support conversion to ndarray
        return self.__points

    def __getitem__(self, indices):
        if not isinstance(indices, tuple) and not isinstance(indices, list):
            # indexing by index or field
            newdata = self.__points[indices]
            if isinstance(indices, str):
                newfields = [dict(self.__fields)[indices]]
            else:
                newfields = self.__fields
        elif all([isinstance(field, str) for field in indices]): # indexing by fields
            newdata = self.__points[list(indices)]
            fdict = dict(self.__fields)
            newfields = [fdict[field] for field in indices]
        elif len(indices) is 2: # indexing by row and col
            if not self.is_organized:
                raise IndexError('only organized point cloud support access by row and column')
            lin = np.arange(_safe_len(self.__points)).reshape(self.width, self.height)
            newdata = self.__points[lin[indices]]
            newfields = self.__fields
        else:
            raise IndexError('too many indices')

        cloud = PointCloud(newdata, fields=newfields, copy=False)
        self.copy_metadata(cloud)
        return cloud

    def __setitem__(self, indices, value):
        if not isinstance(indices, tuple): # indexing by index or field or multiple fields
            self.__points[indices] = value
        elif all([isinstance(field, str) for field in indices]):
            self.__points[list(indices)] = value
        elif len(indices) is 2:
            if not self.is_organized:
                raise IndexError('only organized point cloud support access by row and column')
            lin = np.arange(_safe_len(self.__points)).reshape(self.width, self.height)
            self.__points[lin[indices]] = value
        else:
            raise IndexError('too many indices')

    def __delitem__(self, indices):
        if isinstance(indices, str):
            # delete single field
            nfields = list(self.__fields)
            nfields.remove((indices, dict(nfields)[indices]))
            ndata = np.empty(self.__points.shape, dtype=nfields)
            for name, _ in nfields:
                ndata[name] = self.__points[name]
            self.__points = ndata
            self.__fields = nfields
        elif not isinstance(indices, tuple):
            # delete in one dimension
            self.__points = np.delete(self.__points, indices, axis=0)
        elif all([isinstance(field, str) for field in indices]):
            # delete several fields
            indices = list(indices)
            nfields = list(self.__fields)
            for name in indices:
                nfields.remove((name, dict(nfields)[name]))
            ndata = np.empty(self.__points.shape, dtype=nfields)
            for name, _ in nfields:
                ndata[name] = self.__points[name]
            self.__points = ndata
            self.__fields = nfields
        elif len(indices) is 2:
            # delete in two dimension (organized point cloud)
            if not self.is_organized:
                raise IndexError('only organized point cloud support access by row and column')
            lin = np.arange(_safe_len(self.__points)).reshape(self.width, self.height)
            self.__points = np.delete(self.__points, lin[indices].flatten(), axis=0)
        else:
            raise IndexError('too many indices')

    def __repr__(self):
        return "<PointCloud of %d points>" % _safe_len(self.__points)

    def __len__(self):
        return _safe_len(self.__points)

    def __iter__(self):
        return iter(self.__points)

    def __contains__(self, item):
        # for the field names, use 'names' property for instead.
        return item in self.__points

    def __reduce__(self):
        # Pickle support.
        return type(self), (self.__points, self.__fields, None, self.__width, self.__height,
                            False, self.__sensor_origin, self.__sensor_orientation)

    def __eq__(self, target):
        result = target.data == self.data
        result &= target.sensor_orientation == self.sensor_orientation
        result &= (target.sensor_origin == self.sensor_origin).all()
        result &= target.width == self.width
        result &= target.height == self.height

        if isinstance(result, bool):
            return result
        else: # numpy boolean array
            return result.all()

    def disorganize(self):
        '''
        Disorganize the point cloud. The function can act as updating function
        '''
        self.__width = len(self)
        self.__height = 1

    def insert(self, indices, points):
        '''
        Insert several points into certain location respectively

        # Parameters
            indices: sequence of int
                the indices of the points which determine the location of each point respectively
            points: ndarray with 2-dimension and same point structure with the cloud
                the points that are being inserted
        '''
        points = np.array(points, dtype=self.__fields, copy=False)
        self.__points = np.insert(self.__points, indices, points, axis=0)
        self.disorganize()

    def append(self, points):
        '''
        Insert several points at the end of the container.

        # Parameters
            points: ndarray with 2-dimension and same point structure with the cloud
                the points that are being inserted
        '''
        points = np.array(points, dtype=self.__fields, copy=False)
        self.__points = np.concatenate((self.__points, points), axis=0)
        self.disorganize()

    def pop(self, indices=-1):
        '''
        Remove several points from the cloud.

        # Parameters
            indices: sequence of int
                the indices of the points which determine the location of each point respectively

        # Return
            points: ndarray
                the data of removed points
        '''
        points = np.array(self[indices])
        del self[indices]
        self.disorganize()
        return points

    @property
    def data(self):
        '''
        Get the ndarray storing the points data.
        This property accessor conserves the dtype (data type) of the array
        '''
        return self.__points

    @data.setter
    def data(self, value):
        array = np.array(value, dtype=self.__points.dtype, copy=False) # throw if not compatible
        oldlen = len(self)
        self.__points = array
        if len(self) != oldlen:
            self.disorganize()

    @property
    def width(self):
        '''
        The point cloud width (if organized as an image-structure).

        WIDTH has two meanings:
         - it can specify the total number of points in the cloud
            (equal with POINTS see below) for unorganized datasets;
         - it can specify the width (total number of points in a row)
            of an organized point cloud dataset.
        '''
        return self.__width

    @width.setter
    def width(self, value):
        value = int(value)
        if value < 0 or value > len(self):
            raise ValueError('Invalid input width')
        if len(self) % len != 0:
            raise ValueError('Size of the pointcloud cannot be divided by width')
        self.__width = value
        self.__height = len(self)/value

    @property
    def height(self):
        '''
        The point cloud height (if organized as an image-structure).

        HEIGHT has two meanings:
         - it can specify the height (total number of rows) of an organized point cloud dataset;
         - it is set to 1 for unorganized datasets
            (thus used to check whether a dataset is organized or not).
        '''
        return self.__height

    @height.setter
    def height(self, value):
        value = int(value)
        if value < 0 or value > len(self):
            raise ValueError('Invalid input height')
        if len(self) % len != 0:
            raise ValueError('Size of the pointcloud cannot be divided by height')
        self.__height = value
        self.__width = len(self)/value


    @property
    def is_dense(self):
        '''
        True if no points are invalid (e.g., have NaN or Inf values).
        Tuple types are considered as invalid as well
        '''
        try:
            return not np.isnan(self.__points.tolist()).any()
        except TypeError: # non-number data type
            return False

    @property
    def is_organized(self):
        '''
        Return whether a dataset is organized (e.g., arranged in a structured grid).

        The height value must be different than 1 for a dataset to be organized.
        '''
        return self.__height > 1

    @property
    def sensor_origin(self):
        '''
        Sensor acquisition pose (origin/translation).
        '''
        return self.__sensor_origin

    @sensor_origin.setter
    def sensor_origin(self, value):
        self.__sensor_origin = value

    @property
    def sensor_orientation(self):
        '''
        Sensor acquisition pose (rotation).
        '''
        return self.__sensor_orientation

    @sensor_orientation.setter
    def sensor_orientation(self, value):
        self.__sensor_orientation = value

    def copy_metadata(self, target):
        '''
        Copy metadata of the cloud into target

        Currently, the metadata of the PointCloud is
        - sensor_origin
        - sensor_orientation
        '''
        target.sensor_origin = self.__sensor_origin
        target.sensor_orientation = self.__sensor_orientation

    @property
    def fields(self):
        '''
        Fields of a point in the cloud
        The format of fields is a list of tuples (name, descriptor)
            where the descriptor is nearly the same as numpy
        '''
        return list(self.__fields)

    @property
    def names(self):
        '''
        Get the names of the point fields
        '''
        return [name for name, _ in self.__fields]

    def __str__(self):
        return ('''PointCloud with %d points
width: %d, height: %d
fields: {0}
is_dense: {1}
sensor origin (xyz): {2}
sensor orientation (xyzw) : {3}''' % (len(self), self.width, self.height,)) \
                .format(self.fields, self.is_dense, self.sensor_origin, self.sensor_orientation)

    def append_fields(self, fields, data=None):
        '''
        Append fields at the end of a point

        # Parameters
            fields: list of tuples or numpy.dtype
                The new fields. Format of the fields are the same as init function
            data: number or dict or list or numpy array
                The data correspond to the fields.
                If single number is inputted, then the field will be filled with the number.
                If an array is inputted, the first dimension of the array should match the fields.

                Examples:
                - {'field1': [1, 5], 'field2': [3, 5]}
                - [[1, 5], [3, 5]]
                - array([(1, 3), (5, 5)], dtype=[('field1', 'i1'), ('field2', 'i2')])
                - array([[1, 5], [3, 5]])
        '''
        fields, predict = _cast_fields_to_tuples(fields)
        if predict:
            raise TypeError('Specifying fields of null cloud with only names')
        if len(set([name for name, _ in fields]).intersection(set(self.names))) > 0:
            raise TypeError("Fields with given names already exist.")

        nfields = self.__fields + fields
        ndata = np.empty(self.__points.shape, dtype=nfields)
        for name, _ in self.__fields:
            ndata[name] = self.__points[name]
        if isinstance(data, dict):
            for name, _ in fields:
                if name in data:
                    ndata[name] = data[name]
        elif isinstance(data, np.ndarray) and data.dtype.names is not None:
            for name, _ in fields:
                if name in data.dtype.names:
                    ndata[name] = data[name]
        elif isinstance(data, (list, tuple)) \
            or (isinstance(data, np.ndarray) and data.dtype.names is None):
            for idx, (name, _) in enumerate(fields):
                ndata[name] = data[idx]
        elif data is not None:
            for name, _ in fields:
                ndata[name] = data

        self.__points = ndata
        self.__fields = nfields

    def insert_fields(self, fields, offsets, data=None):
        '''
        Insert fields at given offset in a point

        # Parameters
            fields: list of tuples or numpy.dtype
                The new fields. Format of the fields are the same as init function
            offsets: dict or sequence of int
                The offset where fields are inserted into.
                The offset is defined with field list rather than data bits.

                Examples:
                - {'field1': 2, 'field2': 3}
                - [2, 3]

            data: number or dict or sequence or numpy array
                The data correspond to the fields.
                If single number is inputted, then the field will be filled with the number.
                If an array is inputted, the first dimension of the array should match the fields.

                Examples:
                - {'field1': [1, 5], 'field2': [3, 5]}
                - [[1, 5], [3, 5]]
                - array([(1, 3), (5, 5)], dtype=[('field1', 'i1'), ('field2', 'i2')])
                - array([[1, 5], [3, 5]])
        '''
        fields, predict = _cast_fields_to_tuples(fields)
        if predict:
            raise TypeError('Specifying fields of null cloud with only names')
        if len(set([name for name, _ in fields]).intersection(set(self.names))) > 0:
            raise TypeError("Fields with given names already exist.")

        #Support multiple insert at the same time
        nfields = list(self.__fields)
        nfields.append('TRAILOR')
        if isinstance(offsets, (list, tuple)):
            for idx, offset in enumerate(offsets):
                if isinstance(nfields[offset], list):
                    nfields[offset][-1:-1] = [fields[idx]]
                else:
                    nfields[offset] = [fields[idx], nfields[offset]]
        elif isinstance(offsets, dict):
            for field in fields:
                offset = offsets[field[0]]
                if isinstance(nfields[offset], list):
                    nfields[offset][-1:1] = [field]
                else:
                    nfields[offset] = [field, nfields[offset]]
        temp = []
        for field in nfields:
            if isinstance(field, list):
                temp += field
            else:
                temp.append(field)
        temp.pop() # remove trailor
        nfields = temp

        ndata = np.empty(self.__points.shape, dtype=nfields)
        for name, _ in self.__fields:
            ndata[name] = self.__points[name]
        if isinstance(data, dict):
            for name, _ in fields:
                if name in data:
                    ndata[name] = data[name]
        elif isinstance(data, np.ndarray) and data.dtype.names is not None:
            for name, _ in fields:
                if name in data.dtype.names:
                    ndata[name] = data[name]
        elif isinstance(data, (list, tuple)) \
            or (isinstance(data, np.ndarray) and data.dtype.names is None):
            for idx, (name, _) in enumerate(fields):
                ndata[name] = data[idx]
        elif data is not None:
            for name, _ in fields:
                ndata[name] = data

        self.__points = ndata
        self.__fields = nfields

    def pop_fields(self, names):
        '''
        Remove fields from the pointcloud and return the values respectively

        # Returns
            values: numpy.array
                data of deleted fields, stored in a record array
        '''
        points = np.array(self[names])
        del self[names]
        return points

    def to_ndarray(self, names=None, dtype=None, copy=False):
        '''
        Cast the data into normal numpy.ndarray.
        All the figures in data will be cast to dtype.

        # Parameters
            names: sequence
                Names of the fields that need to be extracted
                If this param is set to None, then all fields will be extracted
            dtype: type or numpy.dtype
                The type that all the figures will be cast to
                If this param is set to None, then it will try to find a proper one
            copy: bool
                Determine whether the function returns a copy or a reference
        '''
        if names is None:
            names = self.names
        data = self.__points[names]
        if dtype is None:
            fdict = dict(self.__fields)
            types = [fdict[name] for name in names]
            if types.count(types[0]) == len(types):
                dtype = types[0] # homogeneous type

        if data.ndim == 0:
            return np.array(data.tolist(), dtype=dtype, copy=copy) # single point (tuple or scalar)
        else:
            return np.array(data.view(dtype).reshape(data.shape + (-1,)), copy=copy)

    @property
    def xyz(self):
        '''
        Get coordinate array of the point cloud, data type will be infered from data

        # Notes
        In PCL, xyz coordinate is stored in data[4] and the last field is filled with 1
        '''
        return self.to_ndarray(['x', 'y', 'z'])

    @property
    def normal(self):
        '''
        Get normal vectors of the point cloud, data type will be infered from data

        # Notes
        In PCL, normals are stored in data_n[4] and the last field is filled with 0
        '''
        return self.to_ndarray(['normal_x', 'normal_y', 'normal_z'])

    @property
    def rgb(self):
        '''
        Get the color field of the pointcloud, the property returns unpacked rgb values
            (float rgb -> uint r,g,b)

        # Notes
        In PCL, rgb is stored as rgba with a=255
        '''
        # struct definition from PCL
        struct = np.dtype([('b', 'i1'), ('g', 'i1'), ('r', 'i1'), ('a', 'i1')])
        return self.data['rgb'].view(struct)[['r', 'g', 'b']]

    @property
    def rgba(self):
        '''
        Get the color field of the pointcloud, the property returns unpacked rgba values
            (float rgb -> uint r,g,b,a)
        '''
        # struct definition from PCL
        struct = np.dtype([('b', 'i1'), ('g', 'i1'), ('r', 'i1'), ('a', 'i1')])
        return self.data['rgba'].view(struct)
