from cython.operator cimport dereference as deref
from libcpp cimport bool

import numpy as np
from pcl.PointCloud cimport PointCloud, _POINT_TYPE_MAPPING
from pcl.common cimport _ensure_zero
from pcl.io.pcd_io cimport loadPCDFile, savePCDFile
from pcl.io.ply_io cimport loadPLYFile, savePLYFile

cdef str _nonzero_error_msg = "Function {0} returned {1}, please check stderr output!"

# TODO: Add checking for whether file exists

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

cpdef PointCloud load_bin(str path, str point_type="xyz"):
    cdef object points = np.fromfile(path, dtype=np.float32)
    cdef PointCloud cloud = PointCloud(points.reshape(-1, len(_POINT_TYPE_MAPPING[point_type.upper().encode('ascii')])), point_type)
    return cloud

cpdef void save_bin(str path, PointCloud cloud):
    cloud.to_ndarray().tofile(path)
