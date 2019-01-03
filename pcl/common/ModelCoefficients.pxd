from libcpp.vector cimport vector
from pcl.common.PCLHeader cimport PCLHeader
from pcl._boost cimport shared_ptr

cdef extern from "pcl/ModelCoefficients.h" namespace "pcl":
    cdef cppclass ModelCoefficients:
        ModelCoefficients ()
        PCLHeader header
        vector[int] values

ctypedef shared_ptr[ModelCoefficients] ModelCoefficientsPtr
ctypedef shared_ptr[const ModelCoefficients] ModelCoefficientsConstPtr
