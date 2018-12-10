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

    def __cinit__(self, init=None):
        self._origin = Vector4f.Zero()
        self._orientation = Quaternionf.Identity()

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
        cdef list dtype = [(field.name, field.datatype) for field in self.fields]
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

