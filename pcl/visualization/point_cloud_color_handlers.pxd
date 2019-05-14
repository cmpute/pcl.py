from libcpp cimport bool
from libcpp.string cimport string
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr

cdef extern from "pcl/visualization/point_cloud_color_handlers.h" namespace "pcl::visualization" nogil:
    cdef cppclass PointCloudColorHandler[PointT]:
        PointCloudColorHandler(const shared_ptr[const PointCloud[PointT]] &cloud)
        bool isCapable ()
        string getName ()
        string getFieldName ()
        # XXX: virtual bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;
        void setInputCloud (const shared_ptr[const PointCloud[PointT]] &cloud)
    cdef cppclass PointCloudColorHandlerRandom[PointT](PointCloudColorHandler[PointT]):
        pass
    cdef cppclass PointCloudColorHandlerCustom[PointT](PointCloudColorHandler[PointT]):
        pass
    cdef cppclass PointCloudColorHandlerRGBField[PointT](PointCloudColorHandler[PointT]):
        pass
    cdef cppclass PointCloudColorHandlerHSVField[PointT](PointCloudColorHandler[PointT]):
        pass
    cdef cppclass PointCloudColorHandlerGenericField[PointT](PointCloudColorHandler[PointT]):
        pass

    cdef cppclass PointCloudColorHandler_PCLPointCloud2 "pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>":
        PointCloudColorHandler_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
        bool isCapable ()
        string getName ()
        string getFieldName ()
        # XXX: virtual bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;
        void setInputCloud (const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudColorHandlerRandom_PCLPointCloud2 "pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>" (PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerRandom_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudColorHandlerCustom_PCLPointCloud2 "pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>" (PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerCustom_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud, double r, double g, double b)
    cdef cppclass PointCloudColorHandlerRGBField_PCLPointCloud2 "pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>" (PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerRGBField_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudColorHandlerHSVField_PCLPointCloud2 "pcl::visualization::PointCloudColorHandlerHSVField<pcl::PCLPointCloud2>" (PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerHSVField_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud)
    cdef cppclass PointCloudColorHandlerGenericField_PCLPointCloud2 "pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>" (PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerGenericField_PCLPointCloud2(const PCLPointCloud2ConstPtr &cloud, const string &field_name)
    
    # TODO: Add details for the handlers
