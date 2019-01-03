from libcpp.string cimport string
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr

cdef extern from "pcl/visualization/point_cloud_handlers.h" namespace "pcl::visualization" nogil:
    cdef cppclass PointCloudGeometryHandler[PointT]:
        PointCloudGeometryHandler(const shared_ptr[const PointCloud[PointT]] &cloud)
    cdef cppclass PointCloudGeometryHandlerXYZ[PointT](PointCloudGeometryHandler[PointT]):
        PointCloudGeometryHandlerXYZ(const shared_ptr[const PointCloud[PointT]] &cloud)
    cdef cppclass PointCloudGeometryHandlerSurfaceNormal[PointT](PointCloudGeometryHandler[PointT]):
        pass
    cdef cppclass PointCloudGeometryHandlerCustom[PointT](PointCloudGeometryHandler[PointT]):
        pass

    cdef cppclass PointCloudGeometryHandler_PCLPointCloud2 "pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>":
        PointCloudGeometryHandler_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudGeometryHandlerXYZ_PCLPointCloud2 "pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>"(PointCloudGeometryHandler_PCLPointCloud2):
        PointCloudGeometryHandlerXYZ_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudGeometryHandlerSurfaceNormal_PCLPointCloud2 "pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2>"(PointCloudGeometryHandler_PCLPointCloud2):
        PointCloudGeometryHandlerSurfaceNormal_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudGeometryHandlerCustom_PCLPointCloud2 "pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>"(PointCloudGeometryHandler_PCLPointCloud2):
        PointCloudGeometryHandlerCustom_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud, const string &x_field_name, const string &y_field_name, const string &z_field_name)

    # TODO: Add details for the handlers
