from libcpp cimport bool
from pcl._boost cimport connection


cdef class BoostConnection:
    cdef connection _connection

    cpdef void disconnect(self)
    cpdef bool connected(self)
    cpdef bool blocked(self)

    @staticmethod
    cdef BoostConnection wrap(const connection& c)
