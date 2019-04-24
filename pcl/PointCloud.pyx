from libc.stdint cimport uint8_t
from libcpp cimport bool
from libcpp.string cimport string
from cython.operator cimport dereference as deref
import sys
import numpy as np
cimport numpy as np

from pcl._boost cimport make_shared
from .ros import ros_exist, ros_error
if ros_exist: from pcl.ros cimport from_msg_cloud, to_msg_cloud
from pcl.common.conversions cimport toPCLPointCloud2, fromPCLPointCloud2
from pcl.common.point_cloud cimport PointCloud as cPC
from pcl.PointField cimport PointField, _FIELD_TYPE_MAPPING

# XXX: unordered_map[name: string, vector[(field_name: string, size_type: uint8_t, count: uint32_t)]]
cdef dict _POINT_TYPE_MAPPING = {
    b'AXIS':    ((b'normal_x',7,1), (b'normal_y',7,1), (b'normal_z',7,1)),
    b'INTENSITY': ((b'intensity',7,1),),
    b'LABLE':   ((b'label',6,1),),
    b'NORMAL':  ((b'normal_x',7,1), (b'normal_y',7,1), (b'normal_z',7,1), (b'curvature',7,1)),
    b'RGB':     ((b'rgba',6,1),),
    b'UV':      ((b'u',7,1), (b'v',7,1)),
    b'XY':      ((b'x',7,1), (b'y',7,1)),
    b'XYZ':     ((b'x',7,1), (b'y',7,1), (b'z',7,1)),
    b'XYZI':    ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'intensity',7,1)),
    b'XYZIN':   ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'intensity',7,1), (b'normal_x',7,1), (b'normal_y',7,1), (b'normal_z',7,1), (b'curvature',7,1)),
    b'XYZL':    ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'label',6,1)),
    b'XYZN':    ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'normal_x',7,1), (b'normal_y',7,1), (b'normal_z',7,1), (b'curvature',7,1)),
    b'XYZRGB':  ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'rgb',7,1)),
    b'XYZRGBA': ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'rgba',6,1)),
    b'XYZRGBL': ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'rgb',7,1), (b'label',6,1)),
    b'XYZRGBN': ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'rgb',7,1), (b'normal_y',7,1), (b'normal_z',7,1), (b'curvature',7,1)),
    b'XYZHSV':  ((b'x',7,1), (b'y',7,1), (b'z',7,1), (b'h',7,1), (b's',7,1), (b'v',7,1)),
}

cdef str _parse_single_dtype(np.dtype dtype):
    # cast single numpy dtype into txt
    if dtype.subdtype is None:
        return dtype.descr[0][1][1:]
    else:
        shape = dtype.subdtype[1][0] if len(dtype.subdtype[1]) == 1 else dtype.subdtype[1]
        return str(shape) + _parse_single_dtype(dtype.subdtype[0])

cdef string _check_dtype_compatible(np.dtype dtype):
    cdef:
        tuple builtin_fields
        tuple builtin_dtypes
        tuple data_fields
        tuple data_dtypes

    for name, typetuple in _POINT_TYPE_MAPPING.items():
        builtin_fields = tuple(name.decode('ascii') for name,_,_ in typetuple)
        builtin_dtypes = tuple(_FIELD_TYPE_MAPPING[typeid][0] for _,typeid,_ in typetuple)
        data_fields = dtype.names
        data_dtypes = tuple(_parse_single_dtype(dtype[name]) for name in dtype.names)
        if builtin_fields == data_fields and builtin_dtypes == data_dtypes:
            return name
    return b"custom"

cdef inline bool _is_not_record_array(np.ndarray array):
    # https://github.com/numpy/numpy/blob/master/numpy/core/_dtype.py#L107
    return array.dtype.fields is None

cdef public class PointCloud[object CyPointCloud, type CyPointCloud_py]:
    def __cinit__(self):
        self._origin = Vector4f.Zero()
        self._orientation = Quaternionf.Identity()

    def __init__(self, data=None, point_type='XYZ'):
        """
        Initialize a point cloud

        # Parameters
        - data: point cloud data, can be a PointCloud or a numpy array. If the dimension of input
            is 3, then it will be considered as dense point cloud with dimensions (height,
            width, data). The input data will always be copied.
        - point_type: if normal numpy array is given, the type of point should be specified 
            if you are using other PCL components on it. The valid point_types are:
            - Intensity
            - Lable
            - Normal
            - RGB
            - UV
            - XY
            - XYZ
            - XYZI
            - XYZIN (PointXYZINormal)
            - XYZL
            - XYZN (PointNormal)
            - XYZRGB
            - XYZRGBA
            - XYZRGBL
            - XYZRGBN (PointXYZRGBNormal)
            - XYZHSV
        """
        # pre-declaration
        cdef PCLPointField temp_field
        cdef bool initialized = False

        if ros_exist:
            from sensor_msgs.msg import PointCloud2
            if isinstance(data, PointCloud2):
                self._ptr = make_shared[PCLPointCloud2]()
                from_msg_cloud(data, deref(self._ptr))
                self.infer_ptype()
                initialized = True
        if isinstance(data, PointCloud):
            self._ptr = make_shared[PCLPointCloud2]()
            self.ptr()[0] = (<PointCloud>data).ptr()[0] # copy struct
            self._origin = (<PointCloud>data)._origin
            self._orientation = (<PointCloud>data)._orientation
            self._ptype = (<PointCloud>data)._ptype
            initialized = True
        if isinstance(data, (list, tuple)):
            if point_type == None:
                raise ValueError('Point type should be specified when normal array is inputed')
            self._ptype = point_type.upper().encode('ascii')
            if <bytes>(self._ptype) not in _POINT_TYPE_MAPPING:
                raise TypeError('Unsupported point type!')

            ndtype = []
            byte_order = '>' if sys.byteorder == 'big' else '<'
            for name, typeid, count in _POINT_TYPE_MAPPING[self._ptype]:
                ndtype.append((name.decode('ascii'),
                    str(count) + byte_order + _FIELD_TYPE_MAPPING[typeid][0]))
            data = np.array(data, dtype=ndtype)
        if isinstance(data, np.ndarray): # TODO: check continuity
            self._ptr = make_shared[PCLPointCloud2]()
            # matrix order detection
            if not data.flags.c_contiguous:
                data = data.ascontiguousarray()

            # datatype interpreting
            if _is_not_record_array(data): # [normal array]
                # data shape interpreting
                if len(data.shape) < 2 or len(data.shape) > 3:
                    raise ValueError("Unrecognized input data shape")

                if len(data.shape) == 2:
                    self.ptr().height = 1
                    self.ptr().width = data.shape[0]
                    self.ptr().is_dense = False
                else:
                    self.ptr().height = data.shape[0]
                    self.ptr().width = data.shape[1]
                    self.ptr().is_dense = True

                # data field interpreting
                if point_type == None:
                    raise ValueError('Point type should be specified when normal array is inputed')
                self._ptype = point_type.upper().encode('ascii')
                if <bytes>(self._ptype) not in _POINT_TYPE_MAPPING:
                    raise TypeError('Unsupported point type!')
                else:
                    # field consensus check
                    if data.shape[-1] != len(_POINT_TYPE_MAPPING[self._ptype]):
                        raise ValueError('Inconsistent field count!')
                    typeid_list = [typeid for _,typeid,_ in _POINT_TYPE_MAPPING[self._ptype]]
                    if typeid_list.count(typeid_list[0]) != len(typeid_list):
                        raise ValueError('This type of point contains different entry types, input\
                                          data is misinterpreted!')
                    if _parse_single_dtype(data.dtype) != _FIELD_TYPE_MAPPING[typeid_list[0]][0]:
                        raise ValueError('Input data is not consistent with what point type requires: '
                                         + _FIELD_TYPE_MAPPING[typeid_list[0]][0])

                # byteorder intepreting
                if (data.dtype.byteorder == '<' and self.ptr().is_bigendian) or\
                   (data.dtype.byteorder == '>' and not self.ptr().is_bigendian):
                    ndtype = data.dtype
                    ndtype.byteorder = '>' if self.ptr().is_bigendian else '<'
                    data = data.astype(ndtype)

            else: # [record array]
                # data shape interpreting
                if len(data.shape) < 1 or len(data.shape) > 2:
                    raise ValueError("Unrecognized input data shape")

                if len(data.shape) == 1:
                    self.ptr().height = 1
                    self.ptr().width = data.shape[0]
                    self.ptr().is_dense = False
                else:
                    self.ptr().height = data.shape[0]
                    self.ptr().width = data.shape[1]
                    self.ptr().is_dense = True

                # data field interpreting
                if self._ptype.empty(): # not previously defined
                    self._ptype = _check_dtype_compatible(data.dtype)
                self.ptr().point_step = data.strides[-1]

                # byteorder intepreting
                ndtype = {'names':[], 'formats':[]}
                for subname in data.dtype.names:
                    subtype = data.dtype[subname]
                    if (subtype.byteorder == '<' and self.ptr().is_bigendian) or\
                       (subtype.byteorder == '>' and not self.ptr().is_bigendian):
                        subtype.byetorder = '>' if self.ptr().is_bigendian else '<'
                    ndtype['names'].append(subname)
                    ndtype['formats'].append(subtype)
                data = data.astype(ndtype)

            # data strides calculation
            if self._ptype != b"custom":
                self.ptr().point_step = 0
                for name, typeid, count in _POINT_TYPE_MAPPING[self._ptype]:
                    temp_field = PCLPointField()
                    temp_field.name = name
                    temp_field.offset = self.ptr().point_step
                    temp_field.datatype = typeid
                    temp_field.count = count
                    self.ptr().fields.push_back(temp_field)
                    self.ptr().point_step += _FIELD_TYPE_MAPPING[typeid][1] * count
                self.ptr().row_step = self.ptr().point_step * self.ptr().width
            else:
                # TODO: Fix

            if not _is_not_record_array(data):
                assert self.ptr().point_step - data.strides[-1] == 0

            # data interpreting
            self.ptr().data = data.view('B').ravel()

            initialized = True
        if data is None:
            self._ptr = make_shared[PCLPointCloud2]()
            initialized = True

        if not initialized:
            raise ValueError("Unrecognized data input!")

    property width:
        '''The width of the point cloud'''
        def __get__(self): return self.ptr().width
    property height:
        '''The height of the point cloud'''
        def __get__(self): return self.ptr().height
    property fields:
        '''The fields of the point cloud'''
        def __get__(self): 
            return [PointField.wrap(field) for field in self.ptr().fields]
    property names:
        '''The name of the point cloud fields'''
        def __get__(self): 
            return [field.name.decode('ascii') for field in self.ptr().fields]
    property sensor_orientation:
        ''' Sensor acquisition pose (rotation) in quaternion (x,y,z,w). '''
        def __get__(self):
            cdef float *mem_ptr = self._orientation.coeffs().data()
            cdef float[:] mem_view = <float[:4]>mem_ptr
            cdef np.ndarray arr_raw = np.asarray(mem_view)
            return arr_raw
        def __set__(self, value):
            self._orientation = Quaternionf(value[3], value[0], value[1], value[2])
    property sensor_origin:
        ''' Sensor acquisition pose (origin/translation). '''
        def __get__(self):
            cdef float *mem_ptr = self._origin.data()
            cdef float[:] mem_view = <float[:3]>mem_ptr
            cdef np.ndarray arr_raw = np.asarray(mem_view)
            return arr_raw
        def __set__(self, value):
            self._origin = Vector4f(value[0], value[1], value[2], 0)

    property xyz:
        '''
        Get coordinate array of the point cloud, data type will be infered from data

        # Notes
        In PCL, xyz coordinate is stored in data[4] and the last field is filled with 1
        '''
        def __get__(self):
            return self.to_ndarray()[['x', 'y', 'z']].view('f4').reshape(len(self), -1)
    property normal:
        '''
        Get normal vectors of the point cloud, data type will be infered from data
        # Notes
        In PCL, normals are stored in data_n[4] and the last field is filled with 0
        '''
        def __get__(self):
            return self.to_ndarray()[['normal_x', 'normal_y', 'normal_z']].view('f4').reshape(len(self), -1)
    property rgb:
        '''
        Get the color field of the pointcloud, the property returns unpacked rgb values
            (float rgb -> uint r,g,b)
        # Notes
        In PCL, rgb is stored as rgba with a=255
        '''
        def __get__(self):
            # struct definition from PCL
            cdef np.dtype struct = np.dtype([('b','u1'), ('g','u1'), ('r','u1'), ('a','u1')])
            return self.to_ndarray()['rgb'].view(struct)[['r', 'g', 'b']]
    property rgba:
        '''
        Get the color field of the pointcloud, the property returns unpacked rgba values
            (float rgb -> uint r,g,b,a)
        '''
        def __get__(self):
            cdef np.dtype struct = np.dtype([('b', 'u1'), ('g', 'u1'), ('r', 'u1'), ('a', 'u1')])
            return self.to_ndarray()['rgba'].view(struct)
    property is_organized:
        '''
        Get whether the point cloud is organized
        '''
        def __get__(self):
            return self.height > 1

    def __len__(self):
        return self.ptr().width * self.ptr().height
    def __repr__(self):
        return "<PointCloud of %d points>" % len(self)
    def __reduce__(self):
        raise NotImplementedError()
    def __iter__(self):
        return iter(self.to_ndarray())
    def __contains__(self, item):
        # for the field names, use 'names' property for instead.
        return item in self.to_ndarray()

    def __getitem__(self, indices):
        if not isinstance(indices, tuple) and not isinstance(indices, list):
            if isinstance(indices, int):
                indices = slice(indices, indices+1) # access one point will return np.void type
            # indexing by index or field
            newdata = self.to_ndarray()[indices]
            if isinstance(indices, str):
                raise NotImplementedError("Accessing by field is currently not supported")
            else:
                newptype = self._ptype
        elif len(indices) is 2: # indexing by row and col
            raise NotImplementedError("Organized cloud is currently not supported")
        else:
            raise IndexError('too many indices')

        cloud = PointCloud(newdata, point_type=newptype.decode('ascii'))
        cloud._origin = self._origin
        cloud._orientation = self._orientation
        return cloud

    def __setitem__(self, indices, value):
        if not isinstance(indices, tuple) and not isinstance(indices, list):
            # indexing by index or field or multiple fields
            self.to_ndarray()[indices] = value
        elif len(indices) is 2:
            raise NotImplementedError("Organized cloud is currently not supported")
        else: raise IndexError('too many indices')

    def __delitem__(self, indices):
        raise NotImplementedError("Delete point is currently not supported")

    def append(self, points):
        copy = self.to_ndarray().copy()
        copy = np.append(copy, np.array(points, dtype=copy.dtype))
        return PointCloud(copy, self._ptype.decode('ascii'))

    def append_fields(self, fields, data):
        raise NotImplementedError('Adding field is currently unsupported')

    def __add__(self, item):
        '''
        Insert several points at the end of the container.
        # Parameters
            points: ndarray with 2-dimension and same point structure with the cloud
                the points that are being inserted
        '''
        if isinstance(item, type(self)) and \
           len(set(self.fields).intersection(item.fields)) == 0:

            if len(item) != len(self):
                raise ValueError('size of point cloud should match when concatenating fields')
            if not self.compare_metadata(item):
                raise ValueError('do not concatenate two point cloud with difference metadata')

            return self.append_fields(None, item)
        else:
            return self.append(item)

    def __eq__(self, PointCloud target):
        result = (target.to_ndarray() == self.to_ndarray()).all()
        result &= self.compare_metadata(target)
        result &= target.width == self.width
        result &= target.height == self.height
        return result

    def __array__(self, *_):
        '''support conversion to ndarray'''
        return self.to_ndarray()
    def __getbuffer__(self, Py_buffer *buffer, int flags):
        raise NotImplementedError()
    def __releasebuffer__(self, Py_buffer *buffer):
        raise NotImplementedError()

    def to_ndarray(self):
        cdef uint8_t *mem_ptr = self.ptr().data.data()
        cdef uint8_t[:] mem_view = <uint8_t[:self.ptr().data.size()]>mem_ptr
        cdef np.ndarray arr_raw = np.asarray(mem_view)
        cdef str byte_order = '>' if self.ptr().is_bigendian else '<'
        cdef np.dtype dtype = np.dtype([(field.name, str(field.count) + byte_order + field.npdtype)
                           for field in self.fields])
        # fix if there is padding bytes. TODO: fix for middle align bytes
        if dtype.itemsize < self.ptr().point_step:
            arr_raw = arr_raw.reshape(-1, self.ptr().point_step)
            arr_raw = np.reshape(arr_raw[:,:dtype.itemsize], -1)
        cdef np.ndarray arr_view = arr_raw.view(dtype)
        return arr_view

    def to_msg(self):
        if not ros_exist: raise ros_error

        import rospy
        cdef object retval = to_msg_cloud(deref(self._ptr))
        retval.header.stamp = rospy.Time.now()
        return retval

    def disorganize(self):
        '''
        Disorganize the point cloud. The function can act as updating function
        '''
        raise NotImplementedError()

    cdef void infer_ptype(self):
        cdef bool field_matched, ptype_matched
        for ptype in _POINT_TYPE_MAPPING.keys():
            ptype_matched = True
            for field in self.ptr().fields:
                field_matched = False
                for fdef in _POINT_TYPE_MAPPING[ptype]:
                    if field.name == bytes(fdef[0]) and field.datatype == fdef[1] and field.count == fdef[2]:
                        field_matched = True
                        break
                if not field_matched:
                    ptype_matched = False
                    break
            if ptype_matched:
                self._ptype = ptype
                return
        self._ptype = b"custom"

    @staticmethod
    cdef PointCloud wrapp(const shared_ptr[PCLPointCloud2]& data):
        cdef PointCloud obj = PointCloud.__new__(PointCloud)
        obj._ptr = data
        obj._origin = Vector4f.Zero()
        obj._orientation = Quaternionf.Identity()
        obj.infer_ptype()
        return obj
