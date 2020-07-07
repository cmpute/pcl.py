from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "pcl/visualization/interactor_style.h" namespace "pcl::visualization":
    cdef cppclass PCLVisualizerInteractorStyle:
        PCLVisualizerInteractorStyle()

        void saveCameraParameters(string file)
        void loadCameraParameters(string file)
        void setCameraFile(string file)
        string getCameraFile()
