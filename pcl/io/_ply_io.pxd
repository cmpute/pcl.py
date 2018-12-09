from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl._point_cloud cimport PointCloud
from pcl._PCLPointCloud2 cimport PCLPointCloud2
from pcl._eigen cimport Vector4f, Quaternionf

cdef extern from "pcl/io/ply_io.h" namespace "pcl::io":
    int loadPLYFile (string &file_name, PCLPointCloud2 &cloud) nogil except +
    int loadPLYFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation) nogil except +
    int loadPLYFile [PointT](string &file_name, PointCloud[PointT] &cloud) nogil except +
    int savePLYFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation, bool binary_mode, bool use_camera) nogil except +
    int savePLYFile [PointT](string &file_name, PointCloud[PointT] &cloud, bool binary_mode) nogil except +
    int savePLYFileASCII [PointT](string &file_name, PointCloud[PointT] &cloud) nogil except +
    int savePLYFileBinary [PointT](string &file_name, PointCloud[PointT] &cloud) nogil except +
    int savePLYFile [PointT](string &file_name, PointCloud[PointT] &cloud, vector[int] &indices, bool binary_mode) nogil except +
    
    # TODO: savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);
