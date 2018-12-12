from libcpp.string cimport string

from pcl._eigen cimport Vector4f, Quaternionf
from pcl._point_cloud cimport PointCloud as cPC
from pcl._PCLPointCloud2 cimport PCLPointCloud2
from pcl._PCLPointField cimport PCLPointField

cdef dict _POINT_TYPE_MAPPING

cdef class PointCloud:
    cdef PCLPointCloud2 _base
    cdef Vector4f _origin
    cdef Quaternionf _orientation
    cdef string _ptype

# cdef inline cPC[PointT] templatize[PointT](PointCloud& data):
#     cdef cPC[PointT] retval
#     fromPCLPointCloud2(data._base, retval)
#     retval.sensor_origin_ = data._origin
#     retval.sensor_orientation_ = data._orientation
#     return retval

# cdef inline PointCloud instantiate[PointT](const cPC[PointT] &data):
#     cdef PointCloud retval
#     toPCLPointCloud2(data, self._base)
#     retval._oridin = data.sensor_origin_
#     retval._orientation = data.sensor_orientation_
#     # TODO: infer type name
#     return retval
