from libcpp.vector cimport vector
from pcl._eigen cimport Matrix4d

cdef extern from "pcl/visualization/common/common.h" namespace "pcl::visualization":
    cdef enum RenderingProperties:
        PCL_VISUALIZER_POINT_SIZE 	
        PCL_VISUALIZER_OPACITY
        PCL_VISUALIZER_LINE_WIDTH
        PCL_VISUALIZER_FONT_SIZE
        PCL_VISUALIZER_COLOR
        PCL_VISUALIZER_REPRESENTATION
        PCL_VISUALIZER_IMMEDIATE_RENDERING
        PCL_VISUALIZER_SHADING

    cdef enum RenderingRepresentationProperties:
        PCL_VISUALIZER_REPRESENTATION_POINTS
        PCL_VISUALIZER_REPRESENTATION_WIREFRAME
        PCL_VISUALIZER_REPRESENTATION_SURFACE

    cdef enum ShadingRepresentationProperties:
        PCL_VISUALIZER_SHADING_FLAT
        PCL_VISUALIZER_SHADING_GOURAUD
        PCL_VISUALIZER_SHADING_PHONG

    cdef cppclass Camera:
        vector[double] focal
        vector[double] pos
        vector[double] view
        double fovy

        vector[double] window_size
        vector[double] window_pos

        void computeViewMatrix(Matrix4d& view_mat)
        void computeProjectionMatrix(Matrix4d& proj)
