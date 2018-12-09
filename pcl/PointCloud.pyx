from libcpp.string cimport string
from cpython cimport bool as PyBool
from pcl._eigen cimport Vector4f, Quaternionf
from pcl._PCLPointCloud2 cimport PCLPointCloud2
from pcl.io._pcd_io cimport loadPCDFile, savePCDFile

cdef class PointCloud:
    cdef PCLPointCloud2 _base
    cdef Vector4f _origin
    cdef Quaternionf _orientation

    def __cinit__(self, init=None):
        self._base = PCLPointCloud2()
        self._origin = Vector4f.Zero() # TODO: default origin and orientation
        self._orientation = Quaternionf()

    def __len__(self):
        return self._base.width

cdef str _nonzero_error_msg = "Function %s returned %d, please check stderr output!"

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

