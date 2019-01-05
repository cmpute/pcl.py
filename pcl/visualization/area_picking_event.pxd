from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from "pcl/visualization/area_picking_event.h" namespace "pcl::visualization":
    cdef cppclass AreaPickingEvent:
        AreaPickingEvent(size_t nb_points, const vector[int] &indices)
        bool getPointsIndices(vector[int] &indices)
