
cdef extern from "vtkSmartPointer.h" nogil:
    cdef cppclass vtkSmartPointer[T]:
        vtkSmartPointer()
        vtkSmartPointer(T*)

        T* Get()
        T* GetPointer()
        T& operator*()
