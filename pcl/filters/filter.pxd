from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.pcl_base cimport PCLBase, PCLBase_PCLPointCloud2
from pcl.common.PointIndices cimport PointIndices, PointIndicesConstPtr

cdef extern from "pcl/filters/filter.h" namespace "pcl":
    cdef cppclass Filter[PointT](PCLBase[PointT]):
        Filter()
        const PointIndicesConstPtr getRemovedIndices()
        void getRemovedIndices(PointIndices &pi)
        void filter(PointCloud[PointT] &output)

    cdef cppclass Filter_PCLPointCloud2 "pcl::Filter<pcl::PCLPointCloud2>"(PCLBase_PCLPointCloud2):
        Filter_PCLPointCloud2()
        const PointIndicesConstPtr getRemovedIndices()
        void getRemovedIndices(PointIndices &pi)
        void filter(PCLPointCloud2 &output)
