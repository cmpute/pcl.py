from cython.operator cimport dereference as deref
from pcl.common.PointIndices cimport PointIndices, PointIndicesPtr

cdef class PCLBase:
    def __init__(self):
        self._ptr = shared_ptr[cPCLBase](NULL)
        self._input = None
        raise NotImplementedError("Abstract class cannot be initialized")

    property InputCloud:
        def __get__(self):
            return self._input
        def __set__(self, PointCloud inputc):
            self._input = inputc
            self._ptr.get().setInputCloud(inputc.csptr())

    property Indices:
        def __get__(self):
            return deref(self._ptr.get().getIndices())
        def __set__(self, indices):
            cdef shared_ptr[vector[int]] vec = make_shared[vector[int]]()
            for idx in indices:
                vec.get().push_back(idx)
            self._ptr.get().setIndices(vec)
