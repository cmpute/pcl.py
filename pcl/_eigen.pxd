# bindings in the package will be finally combined into eigency
# from eigency.core cimport *

cdef extern from "Eigen/Eigen" namespace "Eigen" nogil:
    cdef cppclass Vector4f:
        Vector4f() except +
        Vector4f(float c0, float c1, float c2, float c3) except + 
        float *data()
        float& element "operator()"(int row, int col)

        @staticmethod
        Vector4f Zero()
        
    cdef cppclass Quaternionf:
        Quaternionf()
        Quaternionf(float, float, float, float)
        float w()
        float x()
        float y()
        float z()

        @staticmethod
        Quaternionf Identity()
