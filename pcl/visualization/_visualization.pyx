from libcpp.vector cimport vector
from libcpp.string cimport string
from enum import Enum
import numpy as np
from cython.operator cimport dereference as deref
from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport Affine3f, toAffine3f
from pcl.common cimport _ensure_true
from pcl.common.conversions cimport fromPCLPointCloud2
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.common.point_cloud cimport PointCloud as cPointCloud
from pcl.common.point_types cimport PointXYZ, PointXYZRGB, PointXYZRGBA
from pcl.PointCloud cimport _POINT_TYPE_MAPPING as pmap
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler_PCLPointCloud2, PointCloudGeometryHandlerXYZ_PCLPointCloud2
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2, PointCloudColorHandlerCustom_PCLPointCloud2, PointCloudColorHandlerRGBField_PCLPointCloud2, PointCloudColorHandlerGenericField_PCLPointCloud2
from pcl.visualization._handlers cimport PointCloudColorHandlerPython

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

cdef (unsigned char*) PointCloudColorHandlerCallback(void *func):
    cdef unsigned char [::1] arr
    result = (<object>func)()
    if isinstance(result, np.ndarray):
        if result.shape[-1] != 4:
            raise ValueError("Returned color array should be n*4 shape")
        arr = result.reshape(-1).astype('u1')
    return &arr[0]

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
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewport=0):
        self.ptr().setBackgroundColor(r, g, b, viewport)

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

    cpdef void addCoordinateSystem(self, double scale=1, float x=0, float y=0, float z=0, np.ndarray t=None, int viewport=0):
        cdef Affine3f ct
        if t is not None:
            ct = toAffine3f(t)
            self.ptr().addCoordinateSystem(scale, ct, viewport)
        else:
            self.ptr().addCoordinateSystem(scale, x, y, z, viewport)
    cpdef void removeCoordinateSystem(self, int viewport=0):
        _ensure_true(self.ptr().removeCoordinateSystem(viewport), 'removeCoordinateSystem')

    cpdef void removePointCloud(self, str id="cloud", int viewport=0):
        _ensure_true(self.ptr().removePointCloud(id.encode('ascii'), viewport), 'removePointCloud')
    cpdef void removePolygonMesh(self, str id="polygon", int viewport=0):
        _ensure_true(self.ptr().removePolygonMesh(id.encode('ascii'), viewport), 'removePolygonMesh')
    cpdef void removeShape(self, str id="cloud", int viewport=0):
        _ensure_true(self.ptr().removeShape(id.encode('ascii'), viewport), 'removeShape')
    cpdef void removeText3D(self, str id="cloud", int viewport=0):
        _ensure_true(self.ptr().removeText3D(id.encode('ascii'), viewport), 'removeText3D')
    cpdef void removeAllPointClouds(self, int viewport=0):
        _ensure_true(self.ptr().removeAllPointClouds(viewport), 'removeAllPointClouds')
    cpdef void removeAllShapes(self, int viewport=0):
        _ensure_true(self.ptr().removeAllShapes(viewport), 'removeAllShapes')

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id="", int viewport=0):
        _ensure_true(self.ptr().addText(text.encode('ascii'), xpos, ypos, fontsize, r, g, b, id.encode('ascii'), viewport), 'addText')
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id=""):
        _ensure_true(self.ptr().updateText(text.encode('ascii'), xpos, ypos, fontsize, r, g, b, id.encode('ascii')), 'updateText')
    cpdef void addText3D(self, str text, position, double textScale=1, double r=1, double g=1, double b=1, str id="", int viewport=0):
        '''Note (TODO): This method should be called in the same thread with spin, and viewport seems not to be working'''
        cdef PointXYZ pos = PointXYZ(position[0], position[1], position[2])
        _ensure_true(self.ptr().addText3D[PointXYZ](text.encode('ascii'), pos, textScale, r, g, b, id.encode('ascii'), viewport), 'addText3D')
    cpdef void addPointCloudNormals(self, PointCloud cloud, PointCloud normals=None, int level=100, float scale=0.02, str id="cloud", int viewport=0):
        raise NotImplementedError()
    cpdef void addPointCloudPrincipalCurvatures(self, PointCloud cloud, PointCloud normals, PointCloud pcs, int level=100, float scale=1, str id="cloud", int viewport=0):
        raise NotImplementedError()
    cpdef void addPointCloudIntensityGradients(self, PointCloud cloud, PointCloud gradients, int level=100, double scale=1e-6, str id="cloud", int viewport=0):
        raise NotImplementedError()

    cpdef void addCorrespondences(self, PointCloud source_points, PointCloud target_points, correspondences, str id="correspondences", int viewport=0):
        raise NotImplementedError()
    cpdef void removeCorrespondences (self, str id="correspondences", int viewport=0):
        raise NotImplementedError()

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
    cpdef void createViewPortCamera(self, int viewport):
        self.ptr().createViewPortCamera(viewport)

    cpdef void addPointCloud(self, PointCloud cloud, int r=255, int g=255, int b=255, str field=None, color_handler=None, str id="cloud", int viewport=0):
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler
        cdef shared_ptr[PointCloudColorHandlerRGBField_PCLPointCloud2] rgb_handler
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] mono_handler
        cdef shared_ptr[PointCloudColorHandlerGenericField_PCLPointCloud2] field_handler
        cdef shared_ptr[PointCloudColorHandlerPython] python_handler
        cdef bytes cfield

        fieldlist = [name for name,_,_,_ in pmap[cloud._ptype]]
        if ('rgb' in fieldlist) or ('rgba' in fieldlist):
            xyz_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            rgb_handler = make_shared[PointCloudColorHandlerRGBField_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>rgb_handler,
                cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
                "addPointCloud")
        elif field is not None:
            cfield = field.encode('ascii')
            xyz_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            field_handler = make_shared[PointCloudColorHandlerGenericField_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, cfield)
            _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>field_handler,
                cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
                "addPointCloud")
        elif color_handler is not None: # handler should be callable
            xyz_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            python_handler = shared_ptr[PointCloudColorHandlerPython](new PointCloudColorHandlerPython(<PCLPointCloud2ConstPtr>cloud._ptr, PointCloudColorHandlerCallback, <void*>color_handler))
            _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>python_handler,
                cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
                "addPointCloud")
        else:
            xyz_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            mono_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, r, g, b)
            _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>mono_handler,
                cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
                "addPointCloud")

    cpdef void updatePointCloud(self, PointCloud cloud, str id="cloud"):
        cdef shared_ptr[cPointCloud[PointXYZ]] ccloud_xyz
        cdef shared_ptr[cPointCloud[PointXYZRGBA]] ccloud_rgba

        fieldlist = [name for name,_,_,_ in pmap[cloud._ptype]]
        if ('rgb' in fieldlist) or ('rgba' in fieldlist):
            ccloud_rgba = make_shared[cPointCloud[PointXYZRGBA]]()
            fromPCLPointCloud2(deref(cloud._ptr.get()), deref(ccloud_rgba.get()))
            _ensure_true(self.ptr().updatePointCloud_XYZRGBA(<const shared_ptr[const cPointCloud[PointXYZRGBA]]>ccloud_rgba,
                id.encode('ascii')), "updatePointCloud")
        else:
            ccloud_xyz = make_shared[cPointCloud[PointXYZ]]()
            fromPCLPointCloud2(deref(cloud._ptr.get()), deref(ccloud_xyz.get()))
            _ensure_true(self.ptr().updatePointCloud_XYZ(<const shared_ptr[const cPointCloud[PointXYZ]]>ccloud_xyz,
                id.encode('ascii')), "updatePointCloud")

    cpdef void addLine(self, p1, p2, double r=0.5, double g=0.5, double b=0.5, str id="line", int viewport=0):
        cdef PointXYZ cp1 = PointXYZ(p1[0], p1[1], p1[2])
        cdef PointXYZ cp2 = PointXYZ(p2[0], p2[1], p2[2])
        _ensure_true(self.ptr().addLine(cp1, cp2, r, g, b, id.encode('ascii'), viewport), "addLine")
    cpdef void addArrow(self, p1, p2, double r_line=1, double g_line=1, double b_line=1, double r_text=1, double g_text=1, double b_text=1, bool display_length=False, str id="arrow", int viewport=0):
        cdef PointXYZ cp1 = PointXYZ(p1[0], p1[1], p1[2])
        cdef PointXYZ cp2 = PointXYZ(p2[0], p2[1], p2[2])
        if display_length:
            _ensure_true(self.ptr().addArrow(cp1, cp2, r_line, g_line, b_line, display_length, id.encode('ascii'), viewport), "addArrow")
        else:
            _ensure_true(self.ptr().addArrow(cp1, cp2, r_line, g_line, b_line, r_text, g_text, b_text, id.encode('ascii'), viewport), "addArrow")
    cpdef void addSphere(self, center, double radius, double r=0.5, double g=0.5, double b=0.5, str id="sphere", int viewport=0):
        cdef PointXYZ ctr = PointXYZ(center[0], center[1], center[2])
        _ensure_true(self.ptr().addSphere(ctr, radius, r, g, b, id.encode('ascii'), viewport), "addSphere")
    cpdef void updateSphere(self, center, double radius, double r=0.5, double g=0.5, double b=0.5, str id="sphere"):
        cdef PointXYZ ctr = PointXYZ(center[0], center[1], center[2])
        _ensure_true(self.ptr().updateSphere(ctr, radius, r, g, b, id.encode('ascii')), "updateSphere")
