from libcpp cimport bool
from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.sample_consensus.sac cimport SampleConsensusModel

cdef extern from "pcl/sample_consensus/sac_model_plane.h" namespace "pcl":
    cdef cppclass SampleConsensusModelPlane[PointT](SampleConsensusModel[PointT]):
        SampleConsensusModelPlane()
        # SampleConsensusModelPlane (const PointCloudConstPtr &cloud, bool random = false)
        SampleConsensusModelPlane(const shared_ptr[const PointCloud] &cloud, bool random)
        # SampleConsensusModelPlane (const PointCloudConstPtr &cloud, const std::vector<int> &indices, bool random = false) 
        SampleConsensusModelPlane(const shared_ptr[const PointCloud] &cloud, vector[int] &indices, bool random)
