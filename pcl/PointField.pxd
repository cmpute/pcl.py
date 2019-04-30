from pcl.common.PCLPointField cimport PCLPointField
import numpy as np
cimport numpy as cnp

cdef dict _FIELD_TYPE_MAPPING
cdef dict _FIELD_TYPE_MAPPING_INV

cdef class PointField:
    cdef PCLPointField base
    cdef object __weakref__

    @staticmethod
    cdef PointField wrap(const PCLPointField& data)
