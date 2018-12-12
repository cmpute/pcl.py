from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl.common._point_cloud cimport PointCloud
from pcl.common._PCLPointCloud2 cimport PCLPointCloud2
from pcl._eigen cimport Vector4f, Quaternionf

cdef extern from "pcl/io/pcd_io.h" namespace "pcl::io":
    int loadPCDFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation) nogil except +
    int loadPCDFile [PointT](string &file_name, PointCloud[PointT] &cloud) nogil except +
    int savePCDFile (string &file_name, PCLPointCloud2 &cloud, Vector4f &origin, Quaternionf &orientation, bool binary_mode) nogil except +
    int savePCDFile [PointT](string &file_name, PointCloud[PointT] &cloud, bool binary_mode) nogil except +
    int savePCDFileASCII [PointT](string file_name, PointCloud[PointT] &cloud) nogil except +
    int savePCDFileBinary [PointT](string &file_name, PointCloud[PointT] &cloud) nogil except +
    int savePCDFile [PointT](string &file_name, PointCloud[PointT] &cloud, vector[int] &indices, bool binary_mode) nogil except +
