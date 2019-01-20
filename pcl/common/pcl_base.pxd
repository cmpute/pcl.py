from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2ConstPtr
from pcl.common.PointIndices cimport PointIndicesPtr, PointIndicesConstPtr

cdef extern from "pcl/pcl_base.h" namespace "pcl":
    cdef cppclass PCLBase[PointT]:
        PCLBase()
        void setInputCloud(const shared_ptr[const PointCloud[PointT]] &cloud)
        const shared_ptr[const PointCloud[PointT]] getInputCloud()
        void setIndices(const PointIndicesPtr &indices)
        void setIndices(const PointIndicesConstPtr &indices)
        void setIndices(size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols)
        const PointIndicesPtr getIndices()

    cdef cppclass PCLBase_PCLPointCloud2 "pcl::PCLBase<pcl::PCLPointCloud2>":
        PCLBase_PCLPointCloud2()
        void setInputCloud(const PCLPointCloud2ConstPtr &cloud)
        const PCLPointCloud2ConstPtr getInputCloud()
        void setIndices(const shared_ptr[vector[int]] &indices)
        void setIndices(const PointIndicesConstPtr &indices)
        void setIndices(size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols)
        const shared_ptr[vector[int]] getIndices()
