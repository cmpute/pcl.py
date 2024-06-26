from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl._eigen cimport Vector4f, Quaternionf
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.PolygonMesh cimport PolygonMesh

cdef extern from "pcl/io/ply_io.h" namespace "pcl::io":
    int loadPLYFile (string &file_name, PCLPointCloud2 &cloud) except nogil +
    int loadPLYFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation) except nogil +
    int loadPLYFile[PointT] (string &file_name, PointCloud[PointT] &cloud) except nogil +
    # inline int savePLYFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud, const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (), bool binary_mode = false, bool use_camera = true)
    int savePLYFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation, bool binary_mode, bool use_camera) except nogil +
    int savePLYFile[PointT] (string &file_name, PointCloud[PointT] &cloud, bool binary_mode) except nogil +
    int savePLYFileASCII [PointT](string &file_name, PointCloud[PointT] &cloud) except nogil +
    int savePLYFileBinary [PointT](string &file_name, PointCloud[PointT] &cloud) except nogil +
    # template<typename PointT> int savePLYFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices, bool binary_mode = false)
    int savePLYFile [PointT](string &file_name, PointCloud[PointT] &cloud, vector[int] &indices, bool binary_mode) except nogil +    
    # int savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);
    int savePLYFile (const string &file_name, const PolygonMesh &mesh, unsigned precision)
    int savePLYFileBinary (const string &file_name, const PolygonMesh &mesh)
