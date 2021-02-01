
cdef extern from "pcl/visualization/pcl_visualizer.h" namespace "pcl::visualization":
    cdef cppclass PCLVisualizer_10702 "pcl::visualization::PCLVisualizer":
        # void addCoordinateSystem (double scale = 1.0, const std::string& id = "reference", int viewport = 0);
        void addCoordinateSystem(double scale, const string& id, int viewport)
        # void addCoordinateSystem (double scale, float x, float y, float z, const std::string& id = "reference", int viewport = 0);
        void addCoordinateSystem(double scale, float x, float y, float z, const string& id, int viewport)
        # void addCoordinateSystem (double scale, const Eigen::Affine3f& t, const std::string& id = "reference", int viewport = 0);
        void addCoordinateSystem(double scale, const Affine3f &t, const string& id, int viewport)
        # bool removeCoordinateSystem (const std::string &id = "reference", int viewport = 0);
        bool removeCoordinateSystem(const string& id, int viewport)
