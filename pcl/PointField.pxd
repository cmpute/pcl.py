from pcl._PCLPointField cimport PCLPointField

cdef class PointField:
    cdef PCLPointField base
    cdef object __weakref__

    @staticmethod
    cdef PointField wrap(PCLPointField& data)
