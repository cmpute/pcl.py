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
    cpdef bool keyDown(self)
    cpdef bool keyUp(self)

    @staticmethod
    cdef KeyboardEvent wrap(const cKeyboardEvent& data)

cdef class MouseEvent:
    cdef shared_ptr[cMouseEvent] _ptr
    cdef cMouseEvent* ptr(self)

    @staticmethod
    cdef MouseEvent wrap(const cMouseEvent& data)

cdef class PointPickingEvent:
    cdef shared_ptr[cPointPickingEvent] _ptr
    cdef cPointPickingEvent* ptr(self)

    @staticmethod
    cdef PointPickingEvent wrap(const cPointPickingEvent& data)

cdef class AreaPickingEvent:
    cdef shared_ptr[cAreaPickingEvent] _ptr
    cdef cAreaPickingEvent* ptr(self)

    @staticmethod
    cdef AreaPickingEvent wrap(const cAreaPickingEvent& data)

cdef class Visualizer:
    cdef shared_ptr[PCLVisualizer] _ptr
    cdef object __weakref__
    cdef PCLVisualizer* ptr(self)

    cpdef void setFullScreen(self, bool mode)
    cpdef void setWindowName(self, str name)
    cpdef void setWindowBorders(self, bool mode)
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewport=*)

    # TODO: return wrapped connection object
    cpdef void registerKeyboardCallback(self, callback)
    cpdef void registerMouseCallback(self, callback)
    cpdef void registerPointPickingCallback(self, callback)
    cpdef void registerAreaPickingCallback(self, callback)

    cpdef void spin(self)
    cpdef void spinOnce(self, int time=*, bool force_redraw=*)

    cpdef void addCoordinateSystem(self, double scale=*, float x=*, float y=*, float z=*, np.ndarray t=*, int viewport=*)
    cpdef void removeCoordinateSystem(self, int viewport=*)

    cpdef void removePointCloud(self, str id=*, int viewport=*)
    cpdef void removePolygonMesh(self, str id=*, int viewport=*)
    cpdef void removeShape(self, str id=*, int viewport=*)
    cpdef void removeText3D(self, str id=*, int viewport=*)
    cpdef void removeAllPointClouds(self, int viewport=*)
    cpdef void removeAllShapes(self, int viewport=*)

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=*, double r=*, double g=*, double b=*, str id=*, int viewport=*)
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=*, double r=*, double g=*, double b=*, str id=*)
    cpdef void addText3D(self, str text, position, double textScale=*, double r=*, double g=*, double b=*, str id=*, int viewport=*)
    cpdef void addPointCloudNormals(self, PointCloud cloud, PointCloud normals=*, int level=*, float scale=*, str id=*, int viewport=*)
    cpdef void addPointCloudPrincipalCurvatures(self, PointCloud cloud, PointCloud normals, PointCloud pcs, int level=*, float scale=*, str id=*, int viewport=*)
    cpdef void addPointCloudIntensityGradients(self, PointCloud cloud, PointCloud gradients, int level=*, double scale=*, str id=*, int viewport=*)

    cpdef void addCorrespondences(self, PointCloud source_points, PointCloud target_points, correspondences, str id=*, int viewport=*)
    cpdef void removeCorrespondences (self, str id=*, int viewport=*)

    cpdef void updateShapePose(self, str id, np.ndarray pose)
    cpdef void updatePointCloudPose(self, str id, np.ndarray pose)

    cpdef bool wasStopped(self)
    cpdef void resetStoppedFlag(self)
    cpdef void close(self)
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax)
    cpdef void createViewPortCamera(self, int viewport)

    # TODO: support custom handlers
    cpdef void addPointCloud(self, PointCloud cloud, int r=*, int g=*, int b=*, str field=*, color_handler=*, str id=*, int viewport=*)
    cpdef void updatePointCloud(self, PointCloud cloud, str id=*)

    cpdef void addLine(self, p1, p2, double r=*, double g=*, double b=*, str id=*, int viewport=*)
    cpdef void addArrow(self, p1, p2, double r_line=*, double g_line=*, double b_line=*, double r_text=*, double g_text=*, double b_text=*, bool display_length=*, str id=*, int viewport=*)
    cpdef void addSphere(self, center, double radius, double r=*, double g=*, double b=*, str id=*, int viewport=*)
    cpdef void updateSphere(self, center, double radius, double r=*, double g=*, double b=*, str id=*)
