from pcl._boost cimport shared_ptr
from pcl.sample_consensus.sac cimport SampleConsensus
from pcl.sample_consensus.sac_model cimport SampleConsensusModel

cdef extern from "pcl/sample_consensus/ransac.h" namespace "pcl":
    cdef cppclass RandomSampleConsensus[PointT](SampleConsensus[PointT]):
        RandomSampleConsensus ()
        RandomSampleConsensus (const shared_ptr[SampleConsensusModel[PointT]] &model)
        RandomSampleConsensus (const shared_ptr[SampleConsensusModel[PointT]] &model, double threshold)
