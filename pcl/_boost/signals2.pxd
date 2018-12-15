from libcpp cimport bool

cdef extern from "boost/signals2/connection.hpp" namespace "boost" nogil:
    cdef cppclass connection:
        connection()
        connection(const connection&)
        
        void disconnected()
        bool connected()

        bool blocked()
        void swap(connection&)
