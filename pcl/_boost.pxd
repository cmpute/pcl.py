from libcpp cimport bool

cdef extern from "boost/smart_ptr/shared_ptr.hpp" namespace "boost" nogil:
    cdef cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        shared_ptr(shared_ptr[T]&)
        shared_ptr(shared_ptr[T]&, T*)
        
        # Modifiers
        void reset()
        void reset(T*)
        void swap(shared_ptr&)

        # Observers
        T* get()
        T& operator*()
        bool unique()
        long use_count()
        bool operator bool()
        bool operator!()

        bool operator==(const shared_ptr&)
        bool operator!=(const shared_ptr&)
    
cdef extern from "boost/smart_ptr/make_shared.hpp" namespace "boost" nogil:
    cdef shared_ptr[T] make_shared[T](...)

cdef extern from "boost/signals2/connection.hpp" namespace "boost" nogil:
    cdef cppclass connection:
        connection()
        connection(const connection&)
        
        void disconnected()
        bool connected()

        bool blocked()
        void swap(connection&)
