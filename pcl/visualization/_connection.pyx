from pcl._boost cimport connection

cdef class BoostConnection:
    cpdef void disconnect(self):
        self._connection.disconnect()
    cpdef bool connected(self):
        return self._connection.connected()
    cpdef bool blocked(self):
        return self._connection.blocked()

    @staticmethod
    cdef BoostConnection wrap(const connection& c):
        cdef BoostConnection ret = BoostConnection()
        ret._connection = c
        return ret
