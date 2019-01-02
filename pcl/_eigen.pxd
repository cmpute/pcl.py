# bindings in the package could be finally combined into eigency

cdef extern from "Eigen/Eigen" namespace "Eigen" nogil:
    cdef cppclass Vector4f:
        Vector4f() except +
        Vector4f(float, float, float, float) except + 
        float *data()
        float& element "operator()"(int index)
        @staticmethod
        Vector4f Zero()
        
    cdef cppclass Quaternionf:
        Quaternionf()
        Quaternionf(float w, float x, float y, float z)
        Vector4f& coeffs()
        @staticmethod
        Quaternionf Identity()

    cdef cppclass Affine3f:
        pass
