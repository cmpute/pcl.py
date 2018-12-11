from libc.stdint cimport uint8_t
from libcpp.string cimport string
from cpython cimport bool as PyBool
import numpy as np
cimport numpy as np

from pcl._eigen cimport Vector4f, Quaternionf
from pcl._PCLPointCloud2 cimport PCLPointCloud2
from pcl.io._pcd_io cimport loadPCDFile, savePCDFile
from pcl.PointField cimport PointField

cdef class PointCloud:
    cdef PCLPointCloud2 _base
    cdef Vector4f _origin
    cdef Quaternionf _orientation
    cdef string _ptype

    def __cinit__(self):
        self._origin = Vector4f.Zero()
        self._orientation = Quaternionf.Identity()

    def __init__(self, data=None, point_type=None, copy=False):
        """
        Initialize a point cloud

        Parameters
        ==========
        data: point cloud data, can be a PointCloud or a numpy array. If the dimension of input
            is 3, then it will be considered as dense point cloud with dimensions (height,
            width, data)
        point_type: if normal numpy array is given, the type of point should be specified 
            if you are using other PCL components on it. The valid point_types are:
            - Intensity
            - Lable
            - RGB
            - UV
            - XY
            - XYZ
            - XYZI
            - XYZL
            - XYZRGB
            - XYZRGBA
            - XYZRGBL
            - XYZHSV
        """
        if isinstance(data, PointCloud):
            # TODO: support non-copy instantiation
            self._base = (<PointCloud>data)._base
            self._origin = (<PointCloud>data)._origin
            self._orientation = (<PointCloud>data)._orientation
            self._ptype = (<PointCloud>data).ptype
        elif isinstance(data, np.ndarray):
            # TODO: support copying instantiation
            # datatype interpreting
            if data.dtype.isbuiltin: # normal array
                # data shape check
                if len(data.shape) < 2 or len(data.shape) > 3:
                    raise ValueError("Unrecognized input data type")

                # data shape interpreting
                if len(data.shape) == 2:
                    self._base.height = 1
                    self._base.width = data.shape[0]
                    self._base.is_dense = False
                else:
                    self._base.height = data.shape[0]
                    self._base.width = data.shape[1]
                    self._base.is_dense = True
                self.point_step = data.strides[-1] # TODO: get from fields
                self.row_step = self.point_step * self.width

                # TODO: data field creation
                self._ptype = point_type.encode('ascii')

            else: # record array
                # data shape check
                if len(data.shape) < 1 or len(data.shape) > 2:
                    raise ValueError("Unrecognized input data type")

                # data shape interpreting
                if len(data.shape) == 1:
                    self._base.height = 1
                    self._base.width = data.shape[0]
                    self._base.is_dense = False
                else:
                    self._base.height = data.shape[0]
                    self._base.width = data.shape[1]
                    self._base.is_dense = True

                # TODO: data field creation
                self._ptype = point_type.encode('ascii')

            # steps calculation
            self.point_step = data.strides[-1]
            self.row_step = self.point_step * self.width

            # data interpreting
            if self._base.is_bigendian:
                self._base.data = data.newbyteorder('B')
            else:
                self._base.data = data.newbyteorder('L')

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

    def __len__(self):
        return self._base.width

    def __repr__(self):
        return "<PointCloud of %d points>" % self._base.width

    def to_ndarray(self):
        cdef uint8_t *mem_ptr = self._base.data.data()
        cdef uint8_t[:] mem_view = <uint8_t[:self._base.data.size()]>mem_ptr
        cdef np.ndarray arr_raw = np.asarray(mem_view)
        cdef str byte_order = '>' if self._base.is_bigendian else '<'
        cdef list dtype = [(field.name, byte_order + field.datatype) for field in self.fields]
        cdef np.ndarray arr_view = arr_raw.view(dtype)
        return arr_view

cdef str _nonzero_error_msg = "Function {0} returned {1}, please check stderr output!"

cdef inline bytes _ensure_path(path) except +:
    if isinstance(path, str):
        return path.encode('ascii')
    elif not isinstance(path, bytes):
        raise ValueError('Invalid path string!')
    else: return path

def load_pcd(path):
    cdef PointCloud cloud = PointCloud()
    cdef int retval = loadPCDFile(_ensure_path(path), cloud._base, cloud._origin, cloud._orientation)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("loadPCDFile", retval))
    return cloud

def save_pcd(path, PointCloud cloud, PyBool binary=False):
    _ensure_path(path)
    cdef int retval = savePCDFile(_ensure_path(path), cloud._base, cloud._origin, cloud._orientation, binary)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("savePCDFile", retval))

