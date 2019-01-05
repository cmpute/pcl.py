from libcpp.vector cimport vector
from enum import Enum
from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport Affine3f, toAffine3f
from pcl.common cimport _ensure_true
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler_PCLPointCloud2, PointCloudGeometryHandlerXYZ_PCLPointCloud2
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2, PointCloudColorHandlerCustom_PCLPointCloud2

cdef class KeyboardEvent:
    cdef cKeyboardEvent* ptr(self):
        return self._ptr.get()
    cpdef bool isAltPressed(self):
        return self.ptr().isAltPressed()
    cpdef bool isCtrlPressed(self):
        return self.ptr().isCtrlPressed()
    cpdef bool isShiftPressed(self):
        return self.ptr().isShiftPressed()
    property KeyCode:
        def __get__(self):
            return chr(self.ptr().getKeyCode())
    property KeySym:
        def __get__(self):
            return self.ptr().getKeySym().decode('ascii')
    cpdef bool keyDown(self):
        return self.ptr().keyDown()
    cpdef bool keyUp(self):
        return self.ptr().keyUp()

    @staticmethod
    cdef KeyboardEvent wrap(const cKeyboardEvent& data):
        cdef KeyboardEvent obj = KeyboardEvent.__new__(KeyboardEvent)
        obj._ptr = make_shared[cKeyboardEvent](data)
        return obj

cdef void KeyboardEventCallback(const cKeyboardEvent &event, void *func):
    (<object>func)(KeyboardEvent.wrap(event))

class MouseEvent_Type(Enum):
    MouseMove=1
    MouseButtonPress=2
    MouseButtonRelease=3
    MouseScrollDown=4
    MouseScrollUp=5
    MouseDblClick=6

class MouseEvent_MouseButton(Enum):
    NoButton=0
    LeftButton=1
    MiddleButton=2
    RightButton=3
    VScroll=4

cdef class MouseEvent:
    cdef cMouseEvent* ptr(self):
        return self._ptr.get()
    property Type:
        def __get__(self):
            return MouseEvent_Type(self.ptr().getType())
        def __set__(self, int value):
            self.ptr().setType(<cMouseEvent.Type>value)
    property Button:
        def __get__(self):
            return MouseEvent_MouseButton(self.ptr().getButton())
        def __set__(self, int value):
            self.ptr().setButton(<cMouseEvent.MouseButton>value)
    property X:
        def __get__(self):
            return self.ptr().getX()
    property Y:
        def __get__(self):
            return self.ptr().getY()
    property KeyboardModifiers:
        def __get__(self):
            return self.ptr().getKeyboardModifiers()
    property SelectionMode:
        def __get__(self):
            return self.ptr().getSelectionMode()

    @staticmethod
    cdef MouseEvent wrap(const cMouseEvent& data):
        cdef MouseEvent obj = MouseEvent.__new__(MouseEvent)
        obj._ptr = make_shared[cMouseEvent](data)
        return obj

cdef void MouseEventCallback(const cMouseEvent &event, void *func):
    (<object>func)(MouseEvent.wrap(event))

cdef class PointPickingEvent:
    cdef cPointPickingEvent* ptr(self):
        return self._ptr.get()

    property Point:
        def __get__(self):
            cdef float x=0,y=0,z=0
            self.ptr().getPoint(x,y,z)
            return (x,y,z)
    property PointIndex:
        def __get__(self):
            return self.ptr().getPointIndex()
    property Points:
        def __get__(self):
            cdef float x1=0,y1=0,z1=0,x2=0,y2=0,z2=0
            _ensure_true(self.ptr().getPoints(x1,y1,z1,x2,y2,z2), 'getPoints')
            return [(x1,y1,z1),(x2,y2,z2)]
    property PointIndices:
        def __get__(self):
            cdef int i1=0,i2=0
            _ensure_true(self.ptr().getPointIndices(i1,i2), 'PointIndices')
            return (i1,i2)

    @staticmethod
    cdef PointPickingEvent wrap(const cPointPickingEvent& data):
        cdef PointPickingEvent obj = PointPickingEvent.__new__(PointPickingEvent)
        obj._ptr = make_shared[cPointPickingEvent](data)
        return obj

cdef void PointPickingEventCallback(const cPointPickingEvent &event, void *func):
    (<object>func)(PointPickingEvent.wrap(event))

cdef class AreaPickingEvent:
    cdef cAreaPickingEvent* ptr(self):
        return self._ptr.get()

    property PointsIndices:
        def __get__(self):
            cdef vector[int] indices
            _ensure_true(self.ptr().getPointsIndices(indices), 'getPointsIndices')
            return indices

    @staticmethod
    cdef AreaPickingEvent wrap(const cAreaPickingEvent& data):
        cdef AreaPickingEvent obj = AreaPickingEvent.__new__(AreaPickingEvent)
        obj._ptr = make_shared[cAreaPickingEvent](data)
        return obj

cdef void AreaPickingEventCallback(const cAreaPickingEvent &event, void *func):
    (<object>func)(AreaPickingEvent.wrap(event))

cdef class Visualizer:
    cdef PCLVisualizer* ptr(self):
        return self._ptr.get()
    def __init__(self, str name="", bool create_interactor=True):
        self._ptr = make_shared[PCLVisualizer](<bytes>(name.encode('ascii')), create_interactor)

    cpdef void setFullScreen(self, bool mode):
        self.ptr().setFullScreen(mode)
    cpdef void setWindowName(self, str name):
        self.ptr().setWindowName(name.encode('ascii'))
    cpdef void setWindowBorders(self, bool mode):
        self.ptr().setWindowBorders(mode)
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewpoint=0):
        self.ptr().setBackgroundColor(r, g, b, viewpoint)

    cpdef void registerKeyboardCallback(self, callback):
        self.ptr().registerKeyboardCallback(KeyboardEventCallback, <void*>callback)
    cpdef void registerMouseCallback(self, callback):
        self.ptr().registerMouseCallback(MouseEventCallback, <void*>callback)
    cpdef void registerPointPickingCallback(self, callback):
        self.ptr().registerPointPickingCallback(PointPickingEventCallback, <void*>callback)
    cpdef void registerAreaPickingCallback(self, callback):
        self.ptr().registerAreaPickingCallback(AreaPickingEventCallback, <void*>callback)

    cpdef void spin(self):
        self.ptr().spin()
    cpdef void spinOnce(self, int time=1, bool force_redraw=False):
        self.ptr().spinOnce(time, force_redraw)

    cpdef void addCoordinateSystem(self, double scale=1, float x=0, float y=0, float z=0, np.ndarray t=None, int viewpoint=0):
        cdef Affine3f ct
        if t is not None:
            ct = toAffine3f(t)
            self.ptr().addCoordinateSystem(scale, ct, viewpoint)
        else:
            self.ptr().addCoordinateSystem(scale, x, y, z, viewpoint)
    cpdef void removeCoordinateSystem(self, int viewpoint=0):
        _ensure_true(self.ptr().removeCoordinateSystem(viewpoint), 'removeCoordinateSystem')

    cpdef void removePointCloud(self, str id="cloud", int viewpoint=0):
        _ensure_true(self.ptr().removePointCloud(id.encode('ascii'), viewpoint), 'removePointCloud')
    cpdef void removePolygonMesh(self, str id="polygon", int viewpoint=0):
        _ensure_true(self.ptr().removePolygonMesh(id.encode('ascii'), viewpoint), 'removePolygonMesh')
    cpdef void removeShape(self, str id="cloud", int viewpoint=0):
        _ensure_true(self.ptr().removeShape(id.encode('ascii'), viewpoint), 'removeShape')
    cpdef void removeText3D(self, str id="cloud", int viewpoint=0):
        _ensure_true(self.ptr().removeText3D(id.encode('ascii'), viewpoint), 'removeText3D')
    cpdef void removeAllPointClouds(self, int viewpoint=0):
        _ensure_true(self.ptr().removeAllPointClouds(viewpoint), 'removeAllPointClouds')
    cpdef void removeAllShapes(self, int viewpoint=0):
        _ensure_true(self.ptr().removeAllShapes(viewpoint), 'removeAllShapes')

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id="", int viewpoint=0):
        _ensure_true(self.ptr().addText(text, xpos, ypos, fontsize, r, g, b, id.encode('ascii'), viewpoint), 'addText')
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id=""):
        _ensure_true(self.ptr().updateText(text, xpos, ypos, fontsize, r, g, b, id.encode('ascii')), 'updateText')
    
    cpdef void updateShapePose(self, str id, np.ndarray pose):
        _ensure_true(self.ptr().updateShapePose(id, toAffine3f(pose)), 'updateShapePose')
    cpdef void updatePointCloudPose(self, str id, np.ndarray pose):
        _ensure_true(self.ptr().updatePointCloudPose(id, toAffine3f(pose)), 'updatePointCloudPose')

    cpdef bool wasStopped(self):
        return self.ptr().wasStopped()
    cpdef void resetStoppedFlag(self):
        self.ptr().resetStoppedFlag()
    cpdef void close(self):
        self.ptr().close()
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax):
        cdef int retval = 0
        self.ptr().createViewPort(xmin, ymin, xmax, ymax, retval)
        return retval
    cpdef void createViewPortCamera(self, int viewpoint):
        self.ptr().createViewPortCamera(viewpoint)

    cpdef void addPointCloud(self, PointCloud cloud, str id="cloud", int viewpoint=0):
        # TODO: specialize PointXYZ, PointXYZRGB, PointXYZRGBA
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] geometry_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] color_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, 255, 255, 255)
        _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
            <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>geometry_handler,
            <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>color_handler,
            cloud._origin, cloud._orientation, id.encode('ascii'), viewpoint),
            "addPointCloud")
