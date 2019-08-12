from libc.stdint cimport uint8_t
from libcpp cimport bool
from libcpp.string cimport string
from cpython.object cimport Py_EQ, Py_NE
from cython.operator cimport dereference as deref
import sys
import numpy as np
cimport numpy as np

from pcl._boost cimport make_shared
from .ros import ros_exist, ros_error
if ros_exist: from pcl.ros cimport from_msg_cloud, to_msg_cloud
from pcl.common.conversions cimport toPCLPointCloud2, fromPCLPointCloud2
from pcl.common.point_cloud cimport PointCloud as cPC
from pcl.PointField cimport PointField, _FIELD_TYPE_MAPPING, _FIELD_TYPE_MAPPING_INV

# Type of each item: (field_name: string, size_type: uint8_t, count: uint32_t, tpadding: uint8_t)
cdef tuple UNION_POINT4D    = ((b'x',7,1,0),        (b'y',7,1,0),       (b'z',7,1,4))
cdef tuple UNION_NORMAL4D   = ((b'normal_x',7,1,0), (b'normal_y',7,1,0),(b'normal_z',7,1,4))
cdef tuple UNION_RGB        = ((b'rgba',6,1,0),)

# Dictionary for built-in point types
cdef dict _POINT_TYPE_MAPPING = {
    b'AXIS':    UNION_NORMAL4D,
    b'INTENSITY': ((b'intensity',7,1,0),),
    b'LABEL':   ((b'label',6,1,0),),
    b'NORMAL':  UNION_NORMAL4D + ((b'curvature',7,1,12),),
    b'RGB':     UNION_RGB,
    b'UV':      ((b'u',7,1,0), (b'v',7,1,0)),
    b'XY':      ((b'x',7,1,0), (b'y',7,1,0)),
    b'XYZ':     UNION_POINT4D,
    b'XYZI':    UNION_POINT4D + ((b'intensity',7,1,12),),
    b'XYZIN':   UNION_POINT4D + UNION_NORMAL4D + ((b'intensity',7,1,0), (b'curvature',7,1,8)),
    b'XYZL':    UNION_POINT4D + ((b'label',6,1,0),),
    b'XYZN':    UNION_POINT4D + UNION_NORMAL4D + ((b'curvature',7,1,12),),
    b'XYZRGB':  UNION_POINT4D + UNION_RGB,
    b'XYZRGBA': UNION_POINT4D + UNION_RGB,
    b'XYZRGBL': UNION_POINT4D + UNION_RGB + ((b'label',6,1,0),),
    b'XYZRGBN': UNION_POINT4D + UNION_NORMAL4D + UNION_RGB + ((b'curvature',7,1,8),),
    b'XYZHSV':  UNION_POINT4D + ((b'h',7,1,0), (b's',7,1,0), (b'v',7,1,4)),
}

cdef str _parse_single_dtype(np.dtype dtype):
    # cast single numpy dtype into txt
    if dtype.subdtype is None:
        return dtype.descr[0][1][1:]
    else:
        shape = dtype.subdtype[1][0] if len(dtype.subdtype[1]) == 1 else dtype.subdtype[1]
        return str(shape) + _parse_single_dtype(dtype.subdtype[0])

cdef string _check_dtype_compatible(np.dtype dtype):
    cdef int offset
    cdef bool match_flag

    for name, typetuple in _POINT_TYPE_MAPPING.items():
        dtype_iter = iter(dtype.names)
        match_flag = True

        offset = 0
        for fname, typeid, count, ftpad in typetuple:
            dtname = next(dtype_iter)
            if fname != dtname or _FIELD_TYPE_MAPPING[typeid][0] != _parse_single_dtype(dtype[name])\
                               or offset != dtype.fields[dtname][1]:
                match_flag = False
                break
            if dtype[dtname].subdtype != None:
                if len(dtype[dtname].subdtype[1]) > 1 or dtype[dtname].subdtype[1][0] != count:
                    match_flag = False
                    break
            else:
                if count != 1:
                    match_flag = False
                    break

        if match_flag:
            return name
    return b"CUSTOM"

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
        - data: point cloud data, can be a PointCloud, a numpy array or list of tuples.
            If the dimension of input is 3, then it will be considered as dense point cloud
            with dimensions (height, width, data). The input data will always be copied.
        - point_type: if normal numpy array is given, the type of point should be specified 
            if you are using other PCL components on it. The valid point_types are:
            - Intensity
            - Label
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
            - Custom (Non-PCL point type)

        # Notes
        If the point_type is recognized as built-in types, paddings will be automatically added
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
                raise TypeError('Unsupported point type! You should input a record array if you want custom type.')

            ndtype = {'names':[], 'formats':[], 'offsets':[]}
            offset = 0
            byte_order = '>' if sys.byteorder == 'big' else '<'
            for name, typeid, count, tpad in _POINT_TYPE_MAPPING[self._ptype]:
                tname, tsize = _FIELD_TYPE_MAPPING[typeid]
                ndtype['names'].append(name.decode('ascii'))
                ndtype['formats'].append(str(count) + byte_order + tname)
                ndtype['offsets'].append(offset)
                offset += tsize * count + tpad
            ndtype['itemsize'] = offset
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
                    raise TypeError('Unsupported point type! You should input a record array if you want custom type.')
                else:
                    # field consensus check
                    typeid_list = [typeid for _,typeid,_,_ in _POINT_TYPE_MAPPING[self._ptype]]
                    if typeid_list.count(typeid_list[0]) != len(typeid_list):
                        raise ValueError('This type of point contains different entry types, input'
                                         'data will be misinterpreted!')
                    if _parse_single_dtype(data.dtype) != _FIELD_TYPE_MAPPING[typeid_list[0]][0]:
                        raise ValueError('Input data is not consistent with what point type requires: '
                                         + _FIELD_TYPE_MAPPING[typeid_list[0]][0])
                    size_list = [_FIELD_TYPE_MAPPING[typeid][1]*count*offset 
                        for _,typeid,count,offset in _POINT_TYPE_MAPPING[self._ptype]]
                    if data.strides[-2] != sum(size_list):
                        raise ValueError('Proper padding data are needed for normal array input.'
                                         'Please add padding data or convert the input to list to tuples')

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
                if len(self._ptype) == 0: # not previously defined
                    self._ptype = _check_dtype_compatible(data.dtype)
                self.ptr().point_step = data.strides[-1]

                # byteorder intepreting
                ndtype = {'names':[], 'formats':[], 'offsets':[]}
                for subname in data.dtype.names:
                    subtype, offset = data.dtype.fields[subname]
                    if (subtype.byteorder == '<' and self.ptr().is_bigendian) or\
                       (subtype.byteorder == '>' and not self.ptr().is_bigendian):
                        subtype.byetorder = '>' if self.ptr().is_bigendian else '<'
                    ndtype['names'].append(subname)
                    ndtype['formats'].append(subtype)
                    ndtype['offsets'].append(offset)
                ndtype['itemsize'] = data.dtype.itemsize
                data = data.astype(ndtype)

            # data strides calculation
            if self._ptype != b"CUSTOM":
                self.ptr().point_step = 0
                for name, typeid, count, tpad in _POINT_TYPE_MAPPING[self._ptype]:
                    temp_field = PCLPointField()
                    temp_field.name = name
                    temp_field.offset = self.ptr().point_step
                    temp_field.datatype = typeid
                    temp_field.count = count
                    self.ptr().fields.push_back(temp_field)
                    self.ptr().point_step += _FIELD_TYPE_MAPPING[typeid][1] * count + tpad
                self.ptr().row_step = self.ptr().point_step * self.ptr().width
            else: # only structured array will go into this statement
                for idx in range(len(data.dtype)):
                    if ndtype['formats'][idx].type == np.void: # This field is padding field
                        continue
                    temp_field = PCLPointField()
                    temp_field.name = ndtype['names'][idx].encode('ascii')
                    temp_field.offset = ndtype['offsets'][idx]
                    if ndtype['formats'][idx].subdtype != None:
                        subtype, subcount = ndtype['offsets'].subdtype
                        if len(subcount) > 1:
                            raise ValueError("The input array contain complex field shape which is not supported")
                        temp_field.count = subcount[0]
                        temp_field.datatype = _FIELD_TYPE_MAPPING_INV[_parse_single_dtype(subtype)]
                    else:
                        temp_field.count = 1
                        temp_field.datatype = _FIELD_TYPE_MAPPING_INV[_parse_single_dtype(ndtype['formats'][idx])]
                    self.ptr().fields.push_back(temp_field)
                self.ptr().point_step = ndtype['itemsize']
                self.ptr().row_step = self.ptr().point_step * self.ptr().width

            if _is_not_record_array(data):
                pass
            else:
                assert self.ptr().point_step - data.dtype.itemsize == 0

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

    # The field manipulating is specified to countinuous memory. For details,
    # check https://docs.scipy.org/doc/numpy/user/basics.rec.html#accessing-multiple-fields
    property xyz:
        '''
        Get coordinate array of the point cloud, data type will be infered from data

        # Notes
        In PCL, xyz coordinate is stored in data[4] and the last field is filled with 1

        FIXME: pcl.PointCloud(...).xyz will generate incorrect result!!
        (while cloud = pcl.PointCloud(...); cloud.xyz will return correct one)
        '''
        def __get__(self):
            cdef np.ndarray arr_view = self.to_ndarray()
            
            cdef list offset_check = [arr_view.dtype.fields['x'][1],
                arr_view.dtype.fields['y'][1],
                arr_view.dtype.fields['z'][1]]
            if offset_check != sorted(offset_check):
                raise ValueError("The offsets of x, y, z are not in order and contiguous.")

            ndtype = {'names':['xyz'], 'formats':['3f4'],
                'offsets':[arr_view.dtype.fields['x'][1]], 'itemsize':arr_view.itemsize}

            return arr_view.view(ndtype)['xyz']
    property normal:
        '''
        Get normal vectors of the point cloud, data type will be infered from data
        # Notes
        In PCL, normals are stored in data_n[4] and the last field is filled with 0
        '''
        def __get__(self):
            cdef np.ndarray arr_view = self.to_ndarray()
            
            cdef list offset_check = [arr_view.dtype.fields['normal_x'][1],
                arr_view.dtype.fields['normal_y'][1],
                arr_view.dtype.fields['normal_z'][1]]
            if offset_check != sorted(offset_check):
                raise ValueError("The offsets of normal_x, normal_y, normal_z are not in order and contiguous.")

            ndtype = {'names':['normal_xyz'], 'formats':['3f4'],
                'offsets':[arr_view.dtype.fields['normal_x'][1]], 'itemsize':arr_view.itemsize}

            return arr_view.view(ndtype)['normal_xyz']
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
            return self.to_ndarray()['rgba'].view(struct)[['r', 'g', 'b']]
    property rgba:
        '''
        Get the color field of the pointcloud, the property returns unpacked rgba values
            (float rgb -> uint r,g,b,a)
        '''
        def __get__(self):
            cdef np.dtype struct = np.dtype([('b','u1'), ('g','u1'), ('r','u1'), ('a','u1')])
            return self.to_ndarray()['rgba'].view(struct)
    property is_organized:
        '''
        Get whether the point cloud is organized
        '''
        def __get__(self):
            return self.height > 1
    property ptype:
        def __get__(self):
            return self._ptype.decode("ascii")

    property nptype:
        def __get__(self):
            cdef str byte_order = '>' if self.ptr().is_bigendian else '<'
            cdef dict ndtype = {'names':[], 'formats':[], 'offsets':[]}
            for field in self.fields:
                ndtype['names'].append(field.name)
                ndtype['formats'].append(str(field.count) + byte_order + field.npdtype)
                ndtype['offsets'].append(field.offset)
            ndtype['itemsize'] = self.ptr().point_step
            return np.dtype(ndtype)

    def __len__(self):
        return self.ptr().width * self.ptr().height
    def __repr__(self):
        return "<PointCloud of %d points>" % len(self)
    def __reduce__(self):
        return type(self), (self.to_ndarray(), self._ptype.decode('ascii'))
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
                return PointCloud(newdata)
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
        raise ValueError("Point cloud is immutable")

    def append(self, points):
        if not isinstance(points, np.ndarray):
            points = np.array(points, dtype=self.nptype)
            
        copy = np.append(self.to_ndarray(), points)
        return PointCloud(copy, self._ptype.decode('ascii'))

    def insert(self, points, offsets):
        if not isinstance(points, np.ndarray):
            points = np.array(points, dtype=self.nptype)
        
        copy = np.insert(self.to_ndarray(), offsets, points)
        return PointCloud(copy, self._ptype.decode('ascii'))

    def append_fields(self, data):
        '''
        Append fields at the end of fields list of the point cloud. The new data should have same shape
        with the point cloud data and should have field names predefined in record array

        # Parameters
        data: numpy record array for the new data
        '''
        old_names = self.names
        new_names = data.dtype.names
        if len(set(old_names).intersection(set(new_names))) > 0:
            raise TypeError("fields with given names already exist.")

        npdata = self.to_ndarray()
        ntype = self.nptype.descr + data.dtype.descr
        ndata = np.empty(npdata.shape, dtype=ntype)
        
        for name in old_names:
            ndata[name] = npdata[name]
        for name in new_names:
            ndata[name] = data[name]
        return PointCloud(ndata)

    def insert_fields(self, data, offsets):
        '''
        Insert fields at given offset in the fields list of the cloud

        # Parameters
        data: numpy record array for the new data
        offsets: dict or sequence of int
            The offset where fields are inserted into.
            The offset is defined with field list rather than raw data bits.
            Examples:
            - `{'field1': 2, 'field2': 3}`
            - `[2, 3]`
        '''
        old_names = self.names
        new_names = data.dtype.names
        if len(set(old_names).intersection(set(new_names))) > 0:
            raise TypeError("fields with given names already exist.")

        # Insert fields, support multiple insert at the same time
        npdata = self.to_ndarray()
        ntype = [[field] for field in self.nptype.descr]
        if isinstance(offsets, (list, tuple)):
            for idx, offset in enumerate(offsets):
                ntype[offset].append(data.dtype.descr[idx])
        elif isinstance(offsets, dict):
            for field in data.dtype.descr:
                offset = offsets[field[0]]
                ntype[offset].append(field)
        
        temp = [] # flatten nested ntype
        for flist in ntype:
            for field in reversed(flist):
                temp.append(field)
        ntype = temp

        ndata = np.empty(npdata.shape, dtype=ntype)
        for name in old_names:
            ndata[name] = npdata[name]
        for name in new_names:
            ndata[name] = data[name]      
        return PointCloud(ndata)

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

    def __richcmp__(self, PointCloud target, int op):
        cdef bool result
        result = (target.to_ndarray() == self.to_ndarray()).all()
        result &= self.compare_metadata(target)
        result &= target.width == self.width
        result &= target.height == self.height

        if op == Py_EQ:
            return result
        elif op == Py_NE:
            return not result
        else:
            raise NotImplementedError("Only == and != is supported")

    def __array__(self, *_):
        '''support conversion to ndarray'''
        return self.to_ndarray()
    def __getbuffer__(self, Py_buffer *buffer, int flags):
        raise NotImplementedError()
    def __releasebuffer__(self, Py_buffer *buffer):
        raise NotImplementedError()

    def to_ndarray(self, fields=None):
        cdef uint8_t *mem_ptr = self.ptr().data.data()
        cdef uint8_t[:] mem_view = <uint8_t[:self.ptr().data.size()]>mem_ptr
        cdef np.ndarray arr_raw = np.asarray(mem_view)
        assert not arr_raw.flags['OWNDATA']

        if fields is None:
            ndtype = self.nptype
        else:
            ndtype = np.dtype([field for field in self.nptype.descr if field[0] in fields])
        cdef np.ndarray arr_view = arr_raw.view(ndtype)
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
        self._ptype = b"CUSTOM"

    @staticmethod
    cdef PointCloud wrap(const shared_ptr[PCLPointCloud2]& data):
        cdef PointCloud obj = PointCloud.__new__(PointCloud)
        obj._ptr = data
        obj._origin = Vector4f.Zero()
        obj._orientation = Quaternionf.Identity()
        obj.infer_ptype()
        return obj
