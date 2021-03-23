
cdef extern from "vtkSmartPointer.h" nogil:
    cdef cppclass vtkSmartPointer[T]:
        vtkSmartPointer()
        vtkSmartPointer(T*)

        T* Get()
        T* GetPointer()
        T& operator*()

cdef extern from "vtkRenderWindow.h" nogil:
    cdef cppclass vtkRenderWindow:
        vtkRenderWindow()

        void Render()
