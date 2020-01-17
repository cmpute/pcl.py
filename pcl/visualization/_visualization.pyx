from libc.stdlib cimport malloc, free
from libcpp.vector cimport vector
from libcpp.string cimport string
from cpython.version cimport PY_MAJOR_VERSION
if PY_MAJOR_VERSION == 2:
    from cpython.string cimport PyString_AsString as pystring
elif PY_MAJOR_VERSION == 3:
    from cpython.bytes cimport PyBytes_AsString as pystring
from enum import Enum
import numpy as np
from cython.operator cimport dereference as deref
from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport Vector3f, Quaternionf, Affine3f, toAffine3f
from pcl.common cimport _ensure_true
from pcl.common.conversions cimport fromPCLPointCloud2
from pcl.common.ModelCoefficients cimport ModelCoefficients
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.common.point_cloud cimport PointCloud as cPointCloud
from pcl.common.point_types cimport PointXYZ, PointXYZRGB, PointXYZRGBA
from pcl.PointCloud cimport _POINT_TYPE_MAPPING as pmap
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler_PCLPointCloud2, PointCloudGeometryHandlerXYZ_PCLPointCloud2
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2, PointCloudColorHandlerCustom_PCLPointCloud2, PointCloudColorHandlerRGBField_PCLPointCloud2, PointCloudColorHandlerGenericField_PCLPointCloud2
from pcl.visualization._handlers cimport PointCloudColorHandlerPython
from pcl.visualization.mouse_event_enums cimport Type as cMouseEvent_Type, MouseButton as cMouseEvent_MouseButton
from pcl.visualization.common cimport RenderingProperties as cRenderingProperties,\
    RenderingRepresentationProperties as cRenderingRepresentationProperties,\
    ShadingRepresentationProperties as cShadingRepresentationProperties

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
    MouseMove=cMouseEvent_Type.MouseMove
    MouseButtonPress=cMouseEvent_Type.MouseButtonPress
    MouseButtonRelease=cMouseEvent_Type.MouseButtonRelease
    MouseScrollDown=cMouseEvent_Type.MouseScrollDown
    MouseScrollUp=cMouseEvent_Type.MouseScrollUp
    MouseDblClick=cMouseEvent_Type.MouseDblClick

class MouseEvent_MouseButton(Enum):
    NoButton=cMouseEvent_MouseButton.NoButton
    LeftButton=cMouseEvent_MouseButton.LeftButton
    MiddleButton=cMouseEvent_MouseButton.MiddleButton
    RightButton=cMouseEvent_MouseButton.RightButton
    VScroll=cMouseEvent_MouseButton.VScroll

cdef class MouseEvent:
    cdef cMouseEvent* ptr(self):
        return self._ptr.get()
    property Type:
        def __get__(self):
            return MouseEvent_Type(self.ptr().getType())
        def __set__(self, int value):
            self.ptr().setType(<cMouseEvent_Type>value)
    property Button:
        def __get__(self):
            return MouseEvent_MouseButton(self.ptr().getButton())
        def __set__(self, int value):
            self.ptr().setButton(<cMouseEvent_MouseButton>value)
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

class RenderingProperties(Enum):
    '''
    Point size, integer start from 1
    '''
    PointSize = cRenderingProperties.PCL_VISUALIZER_POINT_SIZE
    '''
    Opacity, 0.0~1.0
    '''
    Opacity = cRenderingProperties.PCL_VISUALIZER_OPACITY
    '''
    Line width, integer start from 1
    '''
    LineWidth = cRenderingProperties.PCL_VISUALIZER_LINE_WIDTH
    '''
    Font size
    '''
    FontSize = cRenderingProperties.PCL_VISUALIZER_FONT_SIZE
    '''
    Color, (R,G,B) tuple with value 0.0~1.0
    '''
    Color = cRenderingProperties.PCL_VISUALIZER_COLOR
    Representation = cRenderingProperties.PCL_VISUALIZER_REPRESENTATION
    ImmediateRendering = cRenderingProperties.PCL_VISUALIZER_IMMEDIATE_RENDERING
    Shading = cRenderingProperties.PCL_VISUALIZER_SHADING

class RenderingRepresentationProperties(Enum):
    Points = cRenderingRepresentationProperties.PCL_VISUALIZER_REPRESENTATION_POINTS
    WireFrame = cRenderingRepresentationProperties.PCL_VISUALIZER_REPRESENTATION_WIREFRAME
    Surface = cRenderingRepresentationProperties.PCL_VISUALIZER_REPRESENTATION_SURFACE

class ShadingRepresentationProperties(Enum):
    Flat = cShadingRepresentationProperties.PCL_VISUALIZER_SHADING_FLAT
    Gouraud = cShadingRepresentationProperties.PCL_VISUALIZER_SHADING_GOURAUD
    Phong = cShadingRepresentationProperties.PCL_VISUALIZER_SHADING_PHONG

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

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=10, color=[1, 1, 1], str id="", int viewport=0):
        _ensure_true(self.ptr().addText(text.encode('ascii'), xpos, ypos, fontsize, color[0], color[1], color[2], id.encode('ascii'), viewport), 'addText')
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=10, color=[1, 1, 1], str id=""):
        _ensure_true(self.ptr().updateText(text.encode('ascii'), xpos, ypos, fontsize, color[0], color[1], color[2], id.encode('ascii')), 'updateText')
    cpdef void addText3D(self, str text, position, double text_scale=1, color=[1, 1, 1], str id="", int viewport=0):
        '''Note (TODO): This method should be called in the same thread with spin, and viewport seems not to be working'''
        cdef PointXYZ pos = PointXYZ(position[0], position[1], position[2])
        _ensure_true(self.ptr().addText3D[PointXYZ](text.encode('ascii'), pos, text_scale, color[0], color[1], color[2], id.encode('ascii'), viewport), 'addText3D')
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

    cpdef void addPointCloud(self, PointCloud cloud, color=[1, 1, 1], str field=None, color_handler=None, str id="cloud", int viewport=0):
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler
        cdef shared_ptr[PointCloudColorHandlerRGBField_PCLPointCloud2] rgb_handler
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] mono_handler
        cdef shared_ptr[PointCloudColorHandlerGenericField_PCLPointCloud2] field_handler
        cdef shared_ptr[PointCloudColorHandlerPython] python_handler
        cdef bytes cfield

        fieldlist = [name for name,_,_,_ in pmap[cloud._ptype]]
        if (b'rgb' in fieldlist) or (b'rgba' in fieldlist):
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
            color_array = np.array(color)
            if np.all(color_array <= 1):
                color_array = np.clip(color_array * 256, 0, 255)

            xyz_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
            mono_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr,
                <int>(color_array[0]), <int>(color_array[1]), <int>(color_array[2]))
            _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>mono_handler,
                cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
                "addPointCloud")

    cpdef void updatePointCloud(self, PointCloud cloud, str id="cloud"):
        cdef shared_ptr[cPointCloud[PointXYZ]] ccloud_xyz
        cdef shared_ptr[cPointCloud[PointXYZRGBA]] ccloud_rgba

        fieldlist = [name for name,_,_,_ in pmap[cloud._ptype]]
        if (b'rgb' in fieldlist) or (b'rgba' in fieldlist):
            ccloud_rgba = make_shared[cPointCloud[PointXYZRGBA]]()
            fromPCLPointCloud2(deref(cloud._ptr.get()), deref(ccloud_rgba.get()))
            _ensure_true(self.ptr().updatePointCloud_XYZRGBA(<const shared_ptr[const cPointCloud[PointXYZRGBA]]>ccloud_rgba,
                id.encode('ascii')), "updatePointCloud")
        else:
            ccloud_xyz = make_shared[cPointCloud[PointXYZ]]()
            fromPCLPointCloud2(deref(cloud._ptr.get()), deref(ccloud_xyz.get()))
            _ensure_true(self.ptr().updatePointCloud_XYZ(<const shared_ptr[const cPointCloud[PointXYZ]]>ccloud_xyz,
                id.encode('ascii')), "updatePointCloud")

    cpdef void addLine(self, p1, p2, color=[0.5, 0.5, 0.5], str id="line", int viewport=0):
        '''
        :param color: (r,g,b) tuple with value 0.0~1.0
        '''
        cdef PointXYZ cp1 = PointXYZ(p1[0], p1[1], p1[2])
        cdef PointXYZ cp2 = PointXYZ(p2[0], p2[1], p2[2])
        _ensure_true(self.ptr().addLine(cp1, cp2, color[0], color[1], color[2], id.encode('ascii'), viewport), "addLine")
    cpdef void addArrow(self, p1, p2, line_color=[1, 1, 1], text_color=[1, 1, 1], bool display_length=False, str id="arrow", int viewport=0):
        cdef PointXYZ cp1 = PointXYZ(p1[0], p1[1], p1[2])
        cdef PointXYZ cp2 = PointXYZ(p2[0], p2[1], p2[2])
        if display_length:
            _ensure_true(self.ptr().addArrow(cp1, cp2, line_color[0], line_color[1], line_color[2],
                display_length, id.encode('ascii'), viewport), "addArrow")
        else:
            _ensure_true(self.ptr().addArrow(cp1, cp2, line_color[0], line_color[1], line_color[2],
                text_color[0], text_color[1], text_color[2], id.encode('ascii'), viewport), "addArrow")
    cpdef void addSphere(self, center, double radius, color=[0.5, 0.5, 0.5], str id="sphere", int viewport=0):
        cdef PointXYZ ctr = PointXYZ(center[0], center[1], center[2])
        _ensure_true(self.ptr().addSphere(ctr, radius, color[0], color[1], color[2], id.encode('ascii'), viewport), "addSphere")
    cpdef void updateSphere(self, center, double radius, color=[0.5, 0.5, 0.5], str id="sphere"):
        cdef PointXYZ ctr = PointXYZ(center[0], center[1], center[2])
        _ensure_true(self.ptr().updateSphere(ctr, radius, color[0], color[1], color[2], id.encode('ascii')), "updateSphere")
    cpdef void addCylinder(self, point_on_axis, axis_direction, double radius, str id="cylinder", int viewport=0):
        cdef ModelCoefficients mcoeff
        mcoeff.values.push_back(point_on_axis[0])
        mcoeff.values.push_back(point_on_axis[1])
        mcoeff.values.push_back(point_on_axis[2])
        mcoeff.values.push_back(axis_direction[0])
        mcoeff.values.push_back(axis_direction[1])
        mcoeff.values.push_back(axis_direction[2])
        mcoeff.values.push_back(radius)
        _ensure_true(self.ptr().addCylinder(mcoeff, id.encode('ascii'), viewport), "addCylinder")
    cpdef void addPlane(self, coeffs, str id="plane", int viewport=0):
        '''
        :param coeffs: the plane coefficients (a, b, c, d with ax+by+cz+d=0)
        '''
        cdef ModelCoefficients mcoeff
        mcoeff.values.push_back(coeffs[0])
        mcoeff.values.push_back(coeffs[1])
        mcoeff.values.push_back(coeffs[2])
        mcoeff.values.push_back(coeffs[3])
        _ensure_true(self.ptr().addPlane(mcoeff, id.encode('ascii'), viewport), "addPlane")
    cpdef void addCircle(self, center, double radius, str id="circle", int viewport=0):
        '''
        Draw 2D circle on the canvas
        :param center: 2D center of the circle
        '''
        cdef ModelCoefficients mcoeff
        mcoeff.values.push_back(center[0])
        mcoeff.values.push_back(center[1])
        mcoeff.values.push_back(radius)
        _ensure_true(self.ptr().addCircle(mcoeff, id.encode('ascii'), viewport), "addCircle")
    cpdef void addCube(self, translation, rotation, double width, double height, double depth, str id="cube", int viewport=0):
        '''
        :param translation: translation in x,y,z
        :param rotation: rotation of quaternion form w,x,y,z
        '''
        cdef Vector3f t = Vector3f(translation[0], translation[1], translation[2])
        cdef Quaternionf r = Quaternionf(rotation[0], rotation[1], rotation[2], rotation[3])
        _ensure_true(self.ptr().addCube(t, r, width, height, depth, id.encode('ascii'), viewport), "addCube")

    cpdef void setShapeRenderingProperties(self, property, value, str id, int viewport=0):
        '''
        :param property: property to be set
        :type property: RenderingProperties
        :param value: value to set as
        '''
        cdef bool result
        if isinstance(property, int):
            property = RenderingProperties(property)
        elif isinstance(property, str):
            property = RenderingProperties[property]

        if property == RenderingProperties.Representation or property == RenderingProperties.Shading:
            result = self.ptr().setShapeRenderingProperties(<int>(property.value), <double>(value.value), id.encode('ascii'), viewport)
        elif property == RenderingProperties.Color:
            result = self.ptr().setShapeRenderingProperties(<int>(property.value), <double>(value[0]), <double>(value[1]), <double>(value[2]), id.encode('ascii'), viewport)
        else:
            result = self.ptr().setShapeRenderingProperties(<int>(property.value), <double>(value), id.encode('ascii'), viewport)
        _ensure_true(result, "setShapeRenderingProperties")
    cpdef void setRepresentationToSurfaceForAllActors(self):
        self.ptr().setRepresentationToSurfaceForAllActors()
    cpdef void setRepresentationToPointsForAllActors(self):
        self.ptr().setRepresentationToPointsForAllActors()
    cpdef void setRepresentationToWireframeForAllActors(self):
        self.ptr().setRepresentationToWireframeForAllActors()

    cpdef void initCameraParameters(self):
        self.ptr().initCameraParameters()
    cpdef void getCameraParameters(self, list argv):
        cdef int argc = len(argv)
        cdef char** array = <char**>malloc(argc * sizeof(char*))
        for i in range(argc):
            array[i] = pystring(argv[i].encode("ascii"))
        self.ptr().getCameraParameters(argc, array)
        free(array)
    cpdef bool cameraParamsSet(self):
        return self.ptr().cameraParamsSet()
    cpdef void updateCamera(self):
        self.ptr().updateCamera()
    cpdef void resetCamera(self):
        self.ptr().resetCamera()
    cpdef void resetCameraViewpoint(self, str id="cloud"):
        self.ptr().resetCameraViewpoint(id=id.encode("ascii"))
    cpdef void setCameraPosition(self, position, view_up, focal_point=None, int viewport=0):
        if not focal_point:
            self.ptr().setCameraPosition(position[0], position[1], position[2],
                view_up[0], view_up[1], view_up[2], viewport)
        else:
            self.ptr().setCameraPosition(position[0], position[1], position[2],
                focal_point[0], focal_point[1], focal_point[2],
                view_up[0], view_up[1], view_up[2], viewport)
    cpdef void setCameraFieldOfView(self, double fovy, int viewport=0):
        self.ptr().setCameraFieldOfView(fovy, viewport)
    cpdef void setCameraClipDistances(self, double near, double far, int viewport=0):
        self.ptr().setCameraClipDistances(near, far, viewport)

    cpdef void setPosition(self, int x, int y):
        self.ptr().setPosition(x, y)
    cpdef void setSize(self, int xw, int yw):
        self.ptr().setSize(xw, yw)
    cpdef void setUseVbos(self, bool use_vbox):
        self.ptr().setUseVbos(use_vbox)
    cpdef void createInteractor(self):
        self.ptr().createInteractor()
    cpdef void saveScreenshot(self, str file):
        self.ptr().saveScreenshot(file.encode("ascii"))
    cpdef void setShowFPS(self, bool show_fps):
        self.ptr().setShowFPS(show_fps)
