from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl._eigen cimport Vector4f, Quaternionf
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2

cdef extern from "pcl/io/pcd_io.h" namespace "pcl::io":
    int loadPCDFile (string &file_name, PCLPointCloud2 &cloud) except nogil +
    int loadPCDFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation) except nogil +
    int loadPCDFile[PointT] (string &file_name, PointCloud[PointT] &cloud) except nogil +
    # inline int savePCDFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud, const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (), const bool binary_mode = false)
    int savePCDFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation, bool binary_mode) except nogil +
    int savePCDFile[PointT] (string &file_name, PointCloud[PointT] &cloud, bool binary_mode) except nogil +
    int savePCDFileASCII[PointT] (string file_name, PointCloud[PointT] &cloud) except nogil +
    int savePCDFileBinary[PointT] (string &file_name, PointCloud[PointT] &cloud) except nogil +
    # template<typename PointT> int savePCDFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices, const bool binary_mode = false)
    int savePCDFile[PointT] (string &file_name, PointCloud[PointT] &cloud, vector[int] &indices, bool binary_mode) except nogil +
    int savePCDFileBinaryCompressed[PointT] (const string &file_name, const PointCloud[PointT] &cloud)

cdef extern from "io_extension.hpp":
    int savePCDFileBinaryCompressed2 (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation) except nogil +
