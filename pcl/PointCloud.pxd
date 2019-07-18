from libcpp.string cimport string

from pcl._boost cimport shared_ptr
from pcl._eigen cimport Vector4f, Quaternionf
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from libcpp cimport bool
from pcl.common.PCLPointField cimport PCLPointField

cdef dict _POINT_TYPE_MAPPING

cdef public class PointCloud[object CyPointCloud, type CyPointCloud_py]:
    cdef shared_ptr[PCLPointCloud2] _ptr
    cdef Vector4f _origin
    cdef Quaternionf _orientation
    cdef string _ptype
    
    cdef inline PCLPointCloud2* ptr(self):
        return self._ptr.get()
    cdef inline shared_ptr[const PCLPointCloud2] csptr(self):
        return <shared_ptr[const PCLPointCloud2]>self._ptr
    cdef inline bool compare_metadata(self, PointCloud target):
        pred = (target.sensor_origin == self.sensor_origin).all()
        pred &= (target.sensor_orientation == self.sensor_orientation).all()
        return pred
    cdef void infer_ptype(self)

    @staticmethod
    cdef PointCloud wrap(const shared_ptr[PCLPointCloud2]& data)
