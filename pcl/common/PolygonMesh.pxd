from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr
from pcl.common.PCLHeader cimport PCLHeader
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.Vertices cimport Vertices

cdef extern from "pcl/PolygonMesh.h" namespace "pcl":
    cdef cppclass PolygonMesh:
        PolygonMesh()
        PCLHeader header
        PCLPointCloud2 cloud
        vector[Vertices] polygons

ctypedef shared_ptr[PolygonMesh] PolygonMeshPtr
ctypedef shared_ptr[const PolygonMesh] PolygonMeshConstPtr
