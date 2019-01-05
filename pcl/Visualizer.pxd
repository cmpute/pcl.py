from libcpp cimport bool
cimport numpy as np
from pcl._boost cimport shared_ptr
from pcl.PointCloud cimport PointCloud
from pcl.visualization.pcl_visualizer cimport PCLVisualizer
from pcl.visualization.area_picking_event cimport AreaPickingEvent as cAreaPickingEvent
from pcl.visualization.keyboard_event cimport KeyboardEvent as cKeyboardEvent
from pcl.visualization.mouse_event cimport MouseEvent as cMouseEvent
from pcl.visualization.point_picking_event cimport PointPickingEvent as cPointPickingEvent

cdef class KeyboardEvent:
    cdef shared_ptr[cKeyboardEvent] _ptr
    cdef cKeyboardEvent* ptr(self)

    cpdef bool isAltPressed(self)
    cpdef bool isCtrlPressed(self)
    cpdef bool isShiftPressed(self)
    cpdef str getKeyCode(self)
    cpdef str getKeySym(self)
    cpdef bool keyDown(self)
    cpdef bool keyUp(self)

    @staticmethod
    cdef KeyboardEvent wrap(const cKeyboardEvent& data)

cdef class Visualizer:
    cdef shared_ptr[PCLVisualizer] _ptr
    cdef object __weakref__
    cdef PCLVisualizer* ptr(self)

    cpdef void setFullScreen(self, bool mode)
    cpdef void setWindowName(self, str name)
    cpdef void setWindowBorders(self, bool mode)
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewpoint=*)

    # TODO: return connection object
    cpdef void registerKeyboardCallback(self, callback)

    cpdef void spin(self)
    cpdef void spinOnce(self, int time=*, bool force_redraw=*)

    cpdef void addCoordinateSystem(self, double scale=*, float x=*, float y=*, float z=*, np.ndarray t=*, int viewpoint=*)
    cpdef void removeCoordinateSystem(self, int viewpoint=*)

    cpdef void removePointCloud(self, str id=*, int viewpoint=*)
    cpdef void removePolygonMesh(self, str id=*, int viewpoint=*)
    cpdef void removeShape(self, str id=*, int viewpoint=*)
    cpdef void removeText3D(self, str id=*, int viewpoint=*)
    cpdef void removeAllPointClouds(self, int viewpoint=*)
    cpdef void removeAllShapes(self, int viewpoint=*)

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=*, double r=*, double g=*, double b=*, str id=*, int viewpoint=*)
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=*, double r=*, double g=*, double b=*, str id=*)
    
    cpdef void updateShapePose(self, str id, np.ndarray pose)
    cpdef void updatePointCloudPose(self, str id, np.ndarray pose)

    cpdef bool wasStopped(self)
    cpdef void resetStoppedFlag(self)
    cpdef void close(self)
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax)
    cpdef void createViewPortCamera(self, int viewport)

    cpdef void addPointCloud(self, PointCloud cloud, str id=*, int viewpoint=*)
    # TODO: cpdef void updatePointCloud(self, PointCloud cloud
