from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.set cimport set as cset
from pcl._boost cimport shared_ptr
from pcl._eigen cimport VectorXf
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.pcl_base cimport PCLBase, PCLBase_PCLPointCloud2
from pcl.common.PointIndices cimport PointIndices, PointIndicesConstPtr
from pcl.sample_consensus.sac_model cimport SampleConsensusModel

cdef extern from "pcl/sample_consensus/sac.h" namespace "pcl":
    cdef cppclass SampleConsensus[PointT]:
        SampleConsensus ()
        SampleConsensus (const shared_ptr[SampleConsensusModel[PointT]] &model, bool random)
        SampleConsensus (const shared_ptr[SampleConsensusModel[PointT]] &model, double threshold, bool random)
        
        void setSampleConsensusModel (const shared_ptr[SampleConsensusModel[PointT]] &model)
        shared_ptr[SampleConsensusModel[PointT]] getSampleConsensusModel ()
        void setDistanceThreshold (double threshold)
        double getDistanceThreshold ()
        void setMaxIterations (int max_iterations)
        int getMaxIterations ()
        void setProbability (double probability)
        double getProbability ()

        ############# abstract methods #############
        bool computeModel (int debug_verbosity_level)
        bool refineModel (const double sigma, const unsigned int max_iterations)
        void getRandomSamples (const shared_ptr [vector[int]] &indices, size_t nr_samples, cset[int] &indices_subset)
        ###########################################

        void getModel (vector[int] &model)
        void getInliers (vector[int] &inliers)
        void getModelCoefficients (VectorXf &model_coefficients)
