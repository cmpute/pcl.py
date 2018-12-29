from libc.stdint cimport uint8_t, uint32_t

cdef extern from "pcl/point_types.h" namespace "pcl":
    cdef cppclass PointXYZ:
        PointXYZ()
        PointXYZ(float x, float y, float z)
        float x
        float y
        float z

    cdef cppclass Normal:
        Normal()
        float normal_x
        float normal_y
        float normal_z
        float curvature

    cdef cppclass PointXYZRGB:
        PointXYZRGB()
        float x
        float y
        float z
        uint8_t b
        uint8_t g
        uint8_t r
        float rgb

    cdef cppclass PointXYZRGBA:
        PointXYZRGBA()
        float x
        float y
        float z
        uint8_t b
        uint8_t g
        uint8_t r
        uint8_t a
        float rgb
        uint32_t rgba

    cdef cppclass PointXYZRGBL:
        PointXYZRGBL()
        float x
        float y
        float z
        uint8_t b
        uint8_t g
        uint8_t r
        float rgb
        uint32_t label

    cdef struct PointXYZI:
        PointXYZI()
        float x
        float y
        float z
        uint32_t label

    cdef struct PrincipalCurvatures:
        PrincipalCurvatures()
        float principal_curvature_x
        float principal_curvature_y
        float principal_curvature_z
        float pc1
        float pc2

    # TODO: include remaining types
