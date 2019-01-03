from cython.operator cimport dereference as deref
from libcpp cimport bool

from pcl.PointCloud cimport PointCloud
from pcl.common cimport _ensure_zero
from pcl.io.pcd_io cimport loadPCDFile, savePCDFile
from pcl.io.ply_io cimport loadPLYFile, savePLYFile

cdef str _nonzero_error_msg = "Function {0} returned {1}, please check stderr output!"

# TODO: Inference point type from fields
cpdef PointCloud load_pcd(str path):
    cdef PointCloud cloud = PointCloud()
    _ensure_zero(loadPCDFile(path.encode('ascii'), deref(cloud._ptr),
        cloud._origin, cloud._orientation), "loadPCDFile")
    return cloud

cpdef void save_pcd(str path, PointCloud cloud, bool binary=False):
    _ensure_zero(savePCDFile(path.encode('ascii'), deref(cloud._ptr),
        cloud._origin, cloud._orientation, binary), "savePCDFile")

# TODO: Inference point type from fields
def load_ply(str path, type return_type=PointCloud):
    cdef PointCloud cloud

    if return_type == PointCloud:
        cloud = PointCloud()
        _ensure_zero(loadPLYFile(path.encode('ascii'), deref(cloud._ptr),
            cloud._origin, cloud._orientation), "loadPLYFile")
        return cloud
    else:
        raise TypeError("Unsupported return type!")

cpdef void save_ply(str path, PointCloud cloud, bool binary=False, bool use_camera=True):
    _ensure_zero(savePLYFile(path.encode('ascii'), deref(cloud._ptr),
        cloud._origin, cloud._orientation, binary, use_camera), "savePLYFile")
