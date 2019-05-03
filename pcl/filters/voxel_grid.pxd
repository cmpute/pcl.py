include "../pcl_config.pxi"
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl._eigen cimport Vector3f, Vector4f, Vector3i, allocator3i
from pcl._boost cimport shared_ptr
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.common.pcl_base cimport PCLBase, PCLBase_PCLPointCloud2
from pcl.common.PointIndices cimport PointIndices, PointIndicesConstPtr
from pcl.filters.filter cimport Filter, Filter_PCLPointCloud2

IF PCL_VER > 107:
    ctypedef vector[Vector3i, allocator3i] VoxelGrid_getNeighborCentroidIndices_type
ELSE:
    ctypedef vector[Vector3i] VoxelGrid_getNeighborCentroidIndices_type

cdef extern from "pcl/filters/voxel_grid.h" namespace "pcl":
    void getMinMax3D (const PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx, Vector4f &min_pt, Vector4f &max_pt)
    # void getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx, const std::string &distance_field_name, float min_distance, float max_distance, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative = false);
    void getMinMax3D (const PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx, const string &distance_field_name, float min_distance, float max_distance, Vector4f &min_pt, Vector4f &max_pt, bool limit_negative)
    void getMinMax3D[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, Vector4f &min_pt, Vector4f &max_pt)
    # void getMinMax3D (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<int> &indices,const std::string &distance_field_name, float min_distance, float max_distance, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative = false);
    void getMinMax3D[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const string &distance_field_name, float min_distance, float max_distance, Vector4f &min_pt, Vector4f &max_pt, bool limit_negative)

    cdef cppclass VoxelGrid[PointT](Filter[PointT]):
        VoxelGrid()
        void setLeafSize(const Vector4f &leaf_size)
        void setLeafSize(float lx, float ly, float lz)
        Vector3f getLeafSize()
        void setDownsampleAllData(bool downsample)
        bool getDownsampleAllData()
        void setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel)
        unsigned int getMinimumPointsNumberPerVoxel() 
        void setSaveLeafLayout (bool save_leaf_layout)
        bool getSaveLeafLayout ()
        Vector3i getMinBoxCoordinates()
        Vector3i getMaxBoxCoordinates()
        Vector3i getNrDivisions()
        Vector3i getDivisionMultiplier()
        int getCentroidIndex (const PointT &p)
        # XXX: vector[int] getNeighborCentroidIndices(const PointT &reference_point, const MatrixXi &relative_coordinates)
        vector[int] getLeafLayout()
        Vector3i getGridCoordinates(float x, float y, float z)
        int getCentroidIndexAt(const Vector3i &ijk)
        void setFilterFieldName (const string &field_name)
        string getFilterFieldName ()
        void setFilterLimits(const double &limit_min, const double &limit_max)
        void getFilterLimits (double &limit_min, double &limit_max)
        void setFilterLimitsNegative (const bool limit_negative)
        void getFilterLimitsNegative (bool &limit_negative)
        bool getFilterLimitsNegative ()

    cdef cppclass VoxelGrid_PCLPointCloud2 "pcl::VoxelGrid<pcl::PCLPointCloud2>"(Filter_PCLPointCloud2):
        VoxelGrid_PCLPointCloud2()
        void setLeafSize(const Vector4f &leaf_size)
        void setLeafSize(float lx, float ly, float lz)
        Vector3f getLeafSize()
        void setDownsampleAllData(bool downsample)
        bool getDownsampleAllData()
        void setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel)
        unsigned int getMinimumPointsNumberPerVoxel() 
        void setSaveLeafLayout (bool save_leaf_layout)
        bool getSaveLeafLayout ()
        Vector3i getMinBoxCoordinates()
        Vector3i getMaxBoxCoordinates()
        Vector3i getNrDivisions()
        Vector3i getDivisionMultiplier()
        int getCentroidIndex (float x, float y, float z)
        # XXX: vector[int] getNeighborCentroidIndices(float x, float y, float z, const MatrixXi &relative_coordinates)
        vector[int] getNeighborCentroidIndices(float x, float y, float z, const VoxelGrid_getNeighborCentroidIndices_type &relative_coordinates)
        vector[int] getLeafLayout()
        Vector3i getGridCoordinates(float x, float y, float z)
        int getCentroidIndexAt(const Vector3i &ijk)
        void setFilterFieldName (const string &field_name)
        string getFilterFieldName ()
        void setFilterLimits (const double &limit_min, const double &limit_max)
        void getFilterLimits (double &limit_min, double &limit_max)
        void setFilterLimitsNegative (const bool limit_negative)
        void getFilterLimitsNegative (bool &limit_negative)
        bool getFilterLimitsNegative ()
