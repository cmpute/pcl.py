from cython.operator cimport dereference as deref
from libcpp cimport bool

from pcl.PointCloud cimport PointCloud
from pcl.io.pcd_io cimport loadPCDFile, savePCDFile
from pcl.io.ply_io cimport loadPLYFile, savePLYFile

cdef str _nonzero_error_msg = "Function {0} returned {1}, please check stderr output!"

# TODO: Inference point type from fields
cpdef PointCloud load_pcd(str path):
    cdef PointCloud cloud = PointCloud()
    cdef int retval = loadPCDFile(path.encode('ascii'), deref(cloud._ptr), cloud._origin, cloud._orientation)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("loadPCDFile", retval))
    return cloud

cpdef void save_pcd(str path, PointCloud cloud, bool binary=False):
    cdef int retval = savePCDFile(path.encode('ascii'), deref(cloud._ptr), cloud._origin, cloud._orientation, binary)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("savePCDFile", retval))

# TODO: Inference point type from fields
def load_ply(str path, type return_type=PointCloud):
    cdef:
        PointCloud cloud
        int retval

    if return_type == PointCloud:
        cloud = PointCloud()
        retval = loadPLYFile(path.encode('ascii'), deref(cloud._ptr), cloud._origin, cloud._orientation)
        if retval != 0: raise RuntimeError(_nonzero_error_msg.format("loadPLYFile", retval))
        return cloud
    else:
        raise TypeError("Unsupported return type!")

cpdef void save_ply(str path, PointCloud cloud, bool binary=False, bool use_camera=True):
    cdef int retval = savePLYFile(path.encode('ascii'), deref(cloud._ptr), cloud._origin, cloud._orientation, binary, use_camera)
    if retval != 0: raise RuntimeError(_nonzero_error_msg.format("savePLYFile", retval))
