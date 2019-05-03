from cython.operator cimport dereference as deref
from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport Vector3i, Vector3f
from pcl.filters.filter cimport Filter_PCLPointCloud2 as cFilter
from pcl.filters.voxel_grid cimport VoxelGrid_getNeighborCentroidIndices_type, VoxelGrid_PCLPointCloud2 as cVoxelGrid
from pcl.PointCloud cimport PointCloud, PCLPointCloud2
from pcl.common.PCLBase cimport PCLBase, cPCLBase

cdef inline cFilter* Filter_ptr(Filter obj):
    return <cFilter*>obj._ptr.get()

cdef class Filter(PCLBase):
    def filter(self):
        cdef PointCloud retval = PointCloud()
        retval._origin = self._input._origin
        retval._orientation = self._input._orientation
        retval._ptype = self._input._ptype

        Filter_ptr(self).filter(deref(retval._ptr))
        return retval

cdef inline cVoxelGrid* VoxelGrid_ptr(VoxelGrid obj):
    return <cVoxelGrid*>(obj._ptr.get())

cdef class VoxelGrid(Filter):
    def __init__(self):
        self._ptr = shared_ptr[cPCLBase](<cPCLBase*>(new cVoxelGrid()))

    property LeafSize:
        def __get__(self):
            cdef Vector3f val = VoxelGrid_ptr(self).getLeafSize()
            return (val.element(0), val.element(1), val.element(2))
        def __set__(self, value):
            VoxelGrid_ptr(self).setLeafSize(value[0], value[1], value[2])

    property DownsampleAllData:
        def __get__(self):
            return VoxelGrid_ptr(self).getDownsampleAllData()
        def __set__(self, value):
            VoxelGrid_ptr(self).setDownsampleAllData(value)

    property MinimumPointsNumberPerVoxel:
        def __get__(self):
            return VoxelGrid_ptr(self).getMinimumPointsNumberPerVoxel()
        def __set__(self, value):
            VoxelGrid_ptr(self).setMinimumPointsNumberPerVoxel(value)

    property SaveLeafLayout:
        def __get__(self):
            return VoxelGrid_ptr(self).getSaveLeafLayout()
        def __set__(self, value):
            VoxelGrid_ptr(self).setSaveLeafLayout(value)

    property MinBoxCoordinates:
        def __get__(self):
            cdef Vector3i val = VoxelGrid_ptr(self).getMinBoxCoordinates()
            return (val.element(0), val.element(1), val.element(2))
    property MaxBoxCoordinates:
        def __get__(self):
            cdef Vector3i val = VoxelGrid_ptr(self).getMaxBoxCoordinates()
            return (val.element(0), val.element(1), val.element(2))
    property NrDivisions:
        def __get__(self):
            cdef Vector3i val = VoxelGrid_ptr(self).getNrDivisions()
            return (val.element(0), val.element(1), val.element(2))
    property DivitionMultiplier:
        def __get__(self):
            cdef Vector3i val = VoxelGrid_ptr(self).getDivisionMultiplier()
            return (val.element(0), val.element(1), val.element(2))

    def getCentroidIndex(self, float x, float y, float z):
        return VoxelGrid_ptr(self).getCentroidIndex(x,y,z)
    def getNeighborCentroidIndices(self, float x, float y, float z, relative_coordinates):
        cdef VoxelGrid_getNeighborCentroidIndices_type coords
        for row in relative_coordinates:
            coords.push_back(Vector3i(row[0], row[1], row[2]))
        return VoxelGrid_ptr(self).getNeighborCentroidIndices(x, y, z, coords)

    property LeafLayout:
        def __get__(self):
            return VoxelGrid_ptr(self).getLeafLayout()

    def getGridCoordinates(self, float x, float y, float z):
        cdef Vector3i retval = VoxelGrid_ptr(self).getGridCoordinates(x, y, z)
        return (retval.element(0), retval.element(1), retval.element(2))
    def getCentroidIndexAt(self, ijk):
        return VoxelGrid_ptr(self).getCentroidIndexAt(Vector3i(ijk[0], ijk[1], ijk[2]))

    property FilterFieldName:
        def __get__(self):
            return VoxelGrid_ptr(self).getFilterFieldName().decode('ascii')
        def __set__(self, value):
            VoxelGrid_ptr(self).setFilterFieldName(value.encode('ascii'))
        
    property FilterLimits:
        def __get__(self):
            cdef double lmin=0, lmax=0
            VoxelGrid_ptr(self).getFilterLimits(lmin, lmax)
            return (lmin, lmax)
        def __set__(self, value):
            VoxelGrid_ptr(self).setFilterLimits(value[0], value[1])
    property FilterLimitsNegative:
        def __get__(self):
            return VoxelGrid_ptr(self).getFilterLimitsNegative()
        def __set__(self, value):
            VoxelGrid_ptr(self).setFilterLimitsNegative(value)
