from libc.stdint cimport uint8_t
from libcpp cimport bool
from libcpp.string cimport string
from cpython cimport bool as PyBool
import numpy as np
cimport numpy as np
import warnings

from pcl._eigen cimport Vector4f, Quaternionf
from pcl.common._conversions cimport toPCLPointCloud2, fromPCLPointCloud2
from pcl.common._PCLPointCloud2 cimport PCLPointCloud2
from pcl.common._PCLPointField cimport PCLPointField
from pcl.common._point_cloud cimport PointCloud as cPC
from pcl.io._pcd_io cimport loadPCDFile, savePCDFile
from pcl.PointField cimport PointField, _FIELD_TYPE_MAPPING

# XXX: unordered_map[string, vector[(string, uint8_t, uint32_t)]]
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

    for name, typetuple in _POINT_TYPE_MAPPING:
        builtin_fields = tuple(name for name,_,_ in typetuple)
        builtin_dtypes = tuple(_FIELD_TYPE_MAPPING[typeid] for _,typeid,_ in typetuple)
        data_fields = dtype.names
        data_dtypes = tuple(_parse_single_dtype(dtype[name]).encode('ascii') for name in dtype.names)
        if builtin_fields == data_fields and builtin_dtypes == data_dtypes:
            return name
    return b""

cdef bool _is_not_record_array(np.ndarray array):
    return array.dtype.isbuiltin

cdef class PointCloud:
    def __cinit__(self):
        self._origin = Vector4f.Zero()
        self._orientation = Quaternionf.Identity()

    def __init__(self, data=None, point_type=None, copy=False):
        """
        Initialize a point cloud

        # Parameters
        - data: point cloud data, can be a PointCloud or a numpy array. If the dimension of input
            is 3, then it will be considered as dense point cloud with dimensions (height,
            width, data)
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

        if isinstance(data, PointCloud):
            # TODO: support non-copy instantiation
            self._base = (<PointCloud>data)._base
            self._origin = (<PointCloud>data)._origin
            self._orientation = (<PointCloud>data)._orientation
            self._ptype = (<PointCloud>data)._ptype
            initialized = True
        if isinstance(data, (list, tuple)):
            data = np.ndarray(data)
        if isinstance(data, np.ndarray):
            # matrix order detection
            if not data.flags.c_contiguous:
                data = data.ascontiguousarray()
                if copy: warnings.warn('The matrix order is inconsistent, data copy is forced!')

            # TODO: support copying instantiation
            # datatype interpreting
            if _is_not_record_array(data): # normal array
                # data shape interpreting
                if len(data.shape) < 2 or len(data.shape) > 3:
                    raise ValueError("Unrecognized input data shape")

                if len(data.shape) == 2:
                    self._base.height = 1
                    self._base.width = data.shape[0]
                    self._base.is_dense = False
                else:
                    self._base.height = data.shape[0]
                    self._base.width = data.shape[1]
                    self._base.is_dense = True

                # data field interpreting
                if point_type == None:
                    raise ValueError('Point type should be specified when normal array is inputed')
                self._ptype = point_type.upper().encode('ascii')
                if <bytes>(self._ptype) not in _POINT_TYPE_MAPPING:
                    warnings.warn('Unsupported point type!')
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

            else: # record array
                # data shape interpreting
                if len(data.shape) < 1 or len(data.shape) > 2:
                    raise ValueError("Unrecognized input data shape")

                if len(data.shape) == 1:
                    self._base.height = 1
                    self._base.width = data.shape[0]
                    self._base.is_dense = False
                else:
                    self._base.height = data.shape[0]
                    self._base.width = data.shape[1]
                    self._base.is_dense = True

                # data field interpreting
                self._ptype = _check_dtype_compatible(data.dtype)
                if self._ptype.empty():
                    raise ValueError("Inconsistent input numpy array dtype!")
                self._base.point_step = data.strides[-1]

            # data strides calculation
            self._base.point_step = 0
            for name, typeid, count in _POINT_TYPE_MAPPING[self._ptype]:
                temp_field = PCLPointField()
                temp_field.name = name
                temp_field.offset = self._base.point_step
                temp_field.datatype = typeid
                temp_field.count = count
                self._base.fields.push_back(temp_field)
                self._base.point_step += _FIELD_TYPE_MAPPING[typeid][1] * count
            self._base.row_step = self._base.point_step * self._base.width

            if not _is_not_record_array(data):
                assert self._base.point_step != data.strides[-1]

            # data interpreting
            # FIXME: generate warning if byte order is not consistent
            if self._base.is_bigendian:
                self._base.data = data.view('>u1').reshape(-1)
            else:
                self._base.data = data.view('<u1').reshape(-1)

            initialized = True
        
        if not initialized:
            raise ValueError("Unrecognized data input!")

    property width:
        '''The width of the point cloud'''
        def __get__(self): return self._base.width
    property height:
        '''The height of the point cloud'''
        def __get__(self): return self._base.height
    property fields:
        '''The fields of the point cloud'''
        def __get__(self): 
            return [PointField.wrap(field) for field in self._base.fields]
    property sensor_orientation:
        ''' Sensor acquisition pose (rotation). '''
        def __get__(self):
            raise NotImplementedError()
    property sensor_origin:
        ''' Sensor acquisition pose (origin/translation). '''
        def __get__(self):
            raise NotImplementedError()

    property xyz:
        '''
        Get coordinate array of the point cloud, data type will be infered from data

        # Notes
        In PCL, xyz coordinate is stored in data[4] and the last field is filled with 1
        '''
        def __get__(self):
            return self.to_ndarray()['x', 'y', 'z']
    property normal:
        '''
        Get normal vectors of the point cloud, data type will be infered from data
        # Notes
        In PCL, normals are stored in data_n[4] and the last field is filled with 0
        '''
        def __get__(self):
            return self.to_ndarray()['normal_x', 'normal_y', 'normal_z']
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

    def __len__(self):
        return self._base.width * self._base.height

    def __repr__(self):
        return "<PointCloud of %d points>" % len(self)

    def __iter__(self):
        raise NotImplementedError()

    def __contains__(self, item):
        raise NotImplementedError()

    def __reduce__(self):
        raise NotImplementedError()

    def to_ndarray(self):
        cdef uint8_t *mem_ptr = self._base.data.data()
        cdef uint8_t[:] mem_view = <uint8_t[:self._base.data.size()]>mem_ptr
        cdef np.ndarray arr_raw = np.asarray(mem_view)
        cdef str byte_order = '>' if self._base.is_bigendian else '<'
        cdef list dtype = [(field.name, str(field.count) + byte_order + field.datatype)
                           for field in self.fields]
        cdef np.ndarray arr_view = arr_raw.view(dtype)
        return arr_view

    def disorganize(self):
        '''
        Disorganize the point cloud. The function can act as updating function
        '''
        raise NotImplementedError()

cdef str _nonzero_error_msg = "Function {0} returned {1}, please check stderr output!"

cdef inline bytes _ensure_path(path) except +:
    if isinstance(path, str):
        return path.encode('ascii')
    elif not isinstance(path, bytes):
        raise ValueError('Invalid path string!')
    else: return path

# TODO: Inference point type from fields
def load_pcd(path):
    cdef PointCloud cloud = PointCloud()
    cdef int retval = loadPCDFile(_ensure_path(path), cloud._base, cloud._origin, cloud._orientation)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("loadPCDFile", retval))
    return cloud

def save_pcd(path, PointCloud cloud, PyBool binary=False):
    _ensure_path(path)
    cdef int retval = savePCDFile(_ensure_path(path), cloud._base, cloud._origin, cloud._orientation, binary)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("savePCDFile", retval))

