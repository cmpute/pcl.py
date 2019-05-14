# This header are aimed to offer minial interface to Eigen
from numpy cimport ndarray

cdef extern from "Eigen/Eigen" namespace "Eigen" nogil:
    cdef cppclass Vector3i:
        Vector3i() except +
        Vector3f(int*) except + 
        Vector3i(int, int, int) except + 
        float *data()
        float& element "operator()"(int index)
        @staticmethod
        Vector3i Zero()

    cdef cppclass Vector3f:
        Vector3f() except +
        Vector3f(float*) except + 
        Vector3f(float, float, float) except + 
        float *data()
        float& element "operator()"(int index)
        @staticmethod
        Vector3f Zero()

    cdef cppclass Vector4f:
        Vector4f() except +
        Vector4f(float*) except + 
        Vector4f(float, float, float, float) except + 
        float *data()
        float& element "operator()"(int index)
        @staticmethod
        Vector4f Zero()

    cdef cppclass VectorXf:
        VectorXf() except +
        VectorXf(size_t) except + 
        VectorXf(float*, size_t) except + 
        float *data()
        float& element "operator()"(int index)
        size_t size()
        @staticmethod
        VectorXf Zero()

    cdef cppclass Matrix3f:
        Matrix3f() except +
        float coeff(int row, int col)
        float *data()
        float& element "operator()"(int row, int col)
        @staticmethod
        Matrix3f Zero()

    cdef cppclass Matrix4f:
        Matrix4f() except +
        float coeff(int row, int col)
        float *data()
        float& element "operator()"(int row, int col)
        @staticmethod
        Matrix4f Zero()
        
    cdef cppclass Quaternionf:
        Quaternionf()
        Quaternionf(float w, float x, float y, float z)
        Vector4f& coeffs()
        @staticmethod
        Quaternionf Identity()

    cdef cppclass Affine3f:
        Affine3f()
        Matrix4f& matrix()
        @staticmethod
        Affine3f Identity()

    cdef cppclass allocator3i "Eigen::aligned_allocator<Eigen::Vector3i>":
        pass

cdef extern from "eigen.hpp" nogil:
    cdef T _toEigen[T,U](U*)
    cdef Affine3f _toEigenAffine(float*)
cdef inline Vector3i toVector3i(ndarray arr):
    return _toEigen[Vector3i, int](<int*>arr.data)
cdef inline Vector3f toVector3f(ndarray arr):
    return _toEigen[Vector3f, float](<float*>arr.data)
cdef inline Vector4f toVector4f(ndarray arr):
    return _toEigen[Vector4f, float](<float*>arr.data)
cdef inline Matrix3f toMatrix3f(ndarray arr):
    return _toEigen[Matrix3f, float](<float*>arr.data)
cdef inline Matrix4f toMatrix4f(ndarray arr):
    return _toEigen[Matrix4f, float](<float*>arr.data)
cdef inline Affine3f toAffine3f(ndarray arr):
    return _toEigenAffine(<float*>arr.data)
