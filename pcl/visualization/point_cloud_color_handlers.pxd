from libcpp cimport bool

cdef extern from "pcl/visualization/point_cloud_color_handlers.h" namespace "pcl::visualization":
    # FIXME: cython unsupported derivation
    cdef cppclass PointCloudColorHandler[PointT]:
        pass
    cdef cppclass PointCloudColorHandlerRandom[PointT](PointCloudColorHandler[PointT]): #(PointCloudColorHander[PointT]):
        pass
    cdef cppclass PointCloudColorHandlerCustom[PointT]:
        pass
    cdef cppclass PointCloudColorHandlerRGBField[PointT]:
        pass
    cdef cppclass PointCloudColorHandlerHSVField[PointT]:
        pass
    cdef cppclass PointCloudColorHandlerGenericField[PointT]:
        pass

    cdef cppclass PointCloudColorHandler_PCLPointCloud2 "PointCloudColorHandler<pcl::PCLPointCloud2>":
        pass
    cdef cppclass PointCloudColorHandlerRandom_PCLPointCloud2(PointCloudColorHandler_PCLPointCloud2) "PointCloudColorHandlerRandom<pcl::PCLPointCloud2>":
        pass
    cdef cppclass PointCloudColorHandlerCustom_PCLPointCloud2(PointCloudColorHandler_PCLPointCloud2) "PointCloudColorHandlerCustom<pcl::PCLPointCloud2>":
        pass
    cdef cppclass PointCloudColorHandlerRGBField_PCLPointCloud2(PointCloudColorHandler_PCLPointCloud2) "PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>":
        pass
    cdef cppclass PointCloudColorHandlerHSVField_PCLPointCloud2(PointCloudColorHandler_PCLPointCloud2) "PointCloudColorHandlerHSVField<pcl::PCLPointCloud2>":
        pass
    cdef cppclass PointCloudColorHandlerGenericField_PCLPointCloud2(PointCloudColorHandler_PCLPointCloud2) "PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>"
        pass
