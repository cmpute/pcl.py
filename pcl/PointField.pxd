from pcl.common.PCLPointField cimport PCLPointField

cdef dict _FIELD_TYPE_MAPPING

cdef class PointField:
    cdef PCLPointField base
    cdef object __weakref__

    @staticmethod
    cdef PointField wrap(const PCLPointField& data)