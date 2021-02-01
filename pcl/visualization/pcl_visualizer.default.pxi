
cdef extern from "pcl/visualization/pcl_visualizer.h" namespace "pcl::visualization":
    cdef cppclass PCLVisualizer_default "pcl::visualization::PCLVisualizer":
        # void addCoordinateSystem (double scale = 1.0, int viewport = 0);
        void addCoordinateSystem(double scale, int viewport)
        # void addCoordinateSystem (double scale, float x, float y, float z, int viewport = 0);
        void addCoordinateSystem(double scale, float x, float y, float z, int viewport)
        # void addCoordinateSystem (double scale, const Eigen::Affine3f& t, int viewport = 0);
        void addCoordinateSystem(double scale, const Affine3f &t, int viewport)
        # bool removeCoordinateSystem (int viewport = 0);
        bool removeCoordinateSystem(int viewport)
