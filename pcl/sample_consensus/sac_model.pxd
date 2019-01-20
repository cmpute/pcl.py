from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.set cimport set as cset
from pcl._eigen cimport VectorXf
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.sample_consensus.model_types cimport SacModel

cdef extern from "pcl/sample_consensus/sac_model.h" namespace "pcl":
    cdef cppclass SampleConsensusModel[PointT]:
        SampleConsensusModel()
        # SampleConsensusModel (const PointCloudConstPtr &cloud, bool random = false)
        SampleConsensusModel(const shared_ptr[const PointCloud[PointT]] &cloud, bool random)
        # SampleConsensusModel (const PointCloudConstPtr &cloud, const std::vector<int> &indices, bool random = false) 
        SampleConsensusModel(const shared_ptr[const PointCloud[PointT]] &cloud, vector[int] &indices, bool random)

        void getSamples (int &iterations, vector[int] &samples)

        ############# abstract methods #############
        bool computeModelCoefficients (const vector[int] &samples, VectorXf &model_coefficients)
        void optimizeModelCoefficients (const vector[int] &inliers, const VectorXf &model_coefficients, VectorXf &optimized_coefficients)
        void getDistancesToModel (const VectorXf &model_coefficients, vector[double] &distances)
        void selectWithinDistance (const VectorXf &model_coefficients, const double threshold, vector[int] &inliers)
        int countWithinDistance (const VectorXf &model_coefficients, const double threshold)
        # void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields = true)
        void projectPoints (const vector[int] &inliers, const VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
        bool doSamplesVerifyModel (const cset[int] &indices, const VectorXf &model_coefficients, const double threshold)
        ###########################################

        void setInputCloud (const shared_ptr[const PointCloud[PointT]] &cloud)
        shared_ptr[const PointCloud[PointT]] getInputCloud ()
        void setIndices (const shared_ptr[vector[int]] &indices)
        void setIndices (const vector[int] &indices)
        shared_ptr[vector[int]] getIndices ()
        SacModel getModelType()
        unsigned int getSampleSize ()
        void setRadiusLimits (const double &min_radius, const double &max_radius)
        void getRadiusLimits (double &min_radius, double &max_radius)
        # XXX: void setSamplesMaxDist (const double &radius, SearchPtr search)
        void getSamplesMaxDist (double &radius)
        double computeVariance (const vector[double] &error_sqr_dists)
        double computeVariance ()
