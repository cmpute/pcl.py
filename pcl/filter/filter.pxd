from pcl.common.pcl_base cimport PCLBase, PCLBase_PCLPointCloud2
from pcl.common.PointIndices cimport PointIndices, PointIndicesConstPtr

cdef extern from "pcl/filter/filter.h" namespace "pcl::filter":
    cdef cppclass Filter[PointT](PCLBase):
        Filter()
        const PointIndicesConstPtr getRemovedIndices()
        void getRemovedIndices(PointIndices &pi)
        void filter(PointCloud &output)

    cdef cppclass Filter_PCLPointCloud2 "pcl::filter::Filter<pcl::PCLPointCloud2>"(PCLBase_PCLPointCloud2):
        Filter_PCLPointCloud2()
        const PointIndicesConstPtr getRemovedIndices()
        void getRemovedIndices(PointIndices &pi)
        void filter(PointCloud &output)
