from libc.stdint cimport uint8_t, uint32_t

cdef extern from "pcl/point_types.h" namespace "pcl":
    cdef struct PointXYZ:
        PointXYZ()
        float x
        float y
        float z

    cdef struct Normal:
        Normal()
        float normal_x
        float normal_y
        float normal_z
        float curvature

    cdef struct PointXYZRGBA:
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

    cdef struct PointXYZRGBL:
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

    # TODO: include remaining types
