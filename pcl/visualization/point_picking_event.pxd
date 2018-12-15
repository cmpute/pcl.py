from libcpp cimport bool

cdef extern from "pcl/visualization/point_picking_event.h" namespace "pcl::visualization":
    cdef cppclass PointPickingEvent:
        PointPickingEvent()
        PointPickingEvent(int idx, float x, float y, float z)
        PointPickingEvent(int idx, int idx2, float x1, float y1, float z1, float x2, float y2, float z2)
        int getPointIndex()
        void getPoint(float &x, float &y, float &z)
        bool getPoints(float &x1, float &y1, float &z1, float &x2, float &y2, float &z2)
        bool getPointIndices(int &index_1, int &index_2)
