from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector

from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.PointIndices cimport PointIndices, PointIndicesConstPtr

cdef extern from "pcl/search/search.h" namespace "pcl":
    cdef cppclass Search[PointT]:
        Search()
        # const std::string& getName () const;
        string getName()
        # void setSortedResults (bool sorted);
        void setSortedResults(bool sorted_)
        # bool getSortedResults ();
        bool getSortedResults()
        # virtual void setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr &indices = IndicesConstPtr ());
        void setInputCloud(const shared_ptr[const PointCloud[PointT]] &cloud, const PointIndicesConstPtr indices)
        # virtual PointCloudConstPtr getInputCloud () const
        const shared_ptr[const PointCloud[PointT]] getInputCloud()
        # virtual IndicesConstPtr getIndices () const
        const PointIndicesPtr getIndices()

        int nearestKSearch(const PointT &point, int k, vector[int] &k_indices, vector[flaot] &k_sqr_distances)
        int nearestKSearchT[PointTDiff](const PointTDiff &point, int k, vector[int] &k_indices, vector[float] &k_sqr_distances)
        int nearestKSearch(const PointCloud[PointT] &cloud, int index, int k, vector[int] &k_indices, vector[float] &k_sqr_distances)
        int nearestKSearch(int index, int k, vector[int] &k_indices, vector[float] &k_sqr_distances)
        int nearestKSearch(const PointCloud[PointT] &cloud, vector[int] indices, int k, vector[int] &k_indices, vector[vector[float]] &k_sqr_distances)
        int nearestKSearchT[PointTDiff](const PointTDiff &point, vector[int] indices, int k, vector[int] &k_indices, vector[vector[float]] &k_sqr_distances)
        
        # virtual int radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances, unsigned int max_nn = 0) const = 0;
        int radiusSearch (const PointT& point, double radius, vector[int]& k_indices, vector[float]& k_sqr_distances, unsigned int max_nn)
        # template <typename PointTDiff> inline int radiusSearchT (const PointTDiff &point, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
        int radiusSearchT[PointTDiff](const PointTDiff &point, double radius, vector[int] &k_indices, vector[] &k_sqr_distances, unsigned int max_nn)
        # virtual int radiusSearch (const PointCloud &cloud, int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;
        int radiusSearch (const PointCloud &cloud, int index, double radius, vector[int] &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn)
        # virtual int radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;
        int radiusSearch (int index, double radius, vector[int] &k_indices, vector[float] &k_sqr_distances, unsigned int max_nn)
        # virtual void radiusSearch (const PointCloud& cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> >& k_indices, std::vector< std::vector<float> > &k_sqr_distances, unsigned int max_nn = 0) const;
        void radiusSearch (const PointCloud[PointT]& cloud, const vector[int]& indices, double radius, vector[vector[int]]& k_indices, vector[vector[float]] &k_sqr_distances, unsigned int max_nn)
        # template <typename PointTDiff> void radiusSearchT (const pcl::PointCloud<PointTDiff> &cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> > &k_indices, std::vector< std::vector<float> > &k_sqr_distances, unsigned int max_nn = 0) const;
        void radiusSearchT[PointTDiff] (const PointCloud[PointTDiff] &cloud, const vector[int]& indices, double radius, vector[vector[int]] &k_indices, vector[vector[float]] &k_sqr_distances, unsigned int max_nn)
