from libcpp.string cimport string

from pcl._eigen cimport Vector4f, Quaternionf
from pcl.common._PCLPointCloud2 cimport PCLPointCloud2
from pcl.common._PCLPointField cimport PCLPointField

cdef dict _POINT_TYPE_MAPPING

cdef public class PointCloud[object CyPointCloud, type CyPointCloud_py]:
    cdef PCLPointCloud2 _base
    cdef Vector4f _origin
    cdef Quaternionf _orientation
    cdef string _ptype

cdef extern from "PointCloud.hpp" namespace "pcl":
    cdef cppclass PointCloudFused
    void CyTemplatize(PointCloud input, const PointCloudFused &output)
    void CyInstantiate(PointCloud output, PointCloudFused &input)
