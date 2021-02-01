from libcpp cimport bool
cimport numpy as np
from pcl._boost cimport shared_ptr
from pcl._vtk cimport vtkSmartPointer
from pcl.PointCloud cimport PointCloud
from pcl.visualization.pcl_visualizer cimport PCLVisualizer
from pcl.visualization.area_picking_event cimport AreaPickingEvent as cAreaPickingEvent
from pcl.visualization.keyboard_event cimport KeyboardEvent as cKeyboardEvent
from pcl.visualization.mouse_event cimport MouseEvent as cMouseEvent
from pcl.visualization.interactor_style cimport PCLVisualizerInteractorStyle
from pcl.visualization.point_picking_event cimport PointPickingEvent as cPointPickingEvent

cdef class KeyboardEvent:
    cdef shared_ptr[cKeyboardEvent] _ptr
    cdef inline cKeyboardEvent* ptr(self)

    cpdef bool isAltPressed(self)
    cpdef bool isCtrlPressed(self)
    cpdef bool isShiftPressed(self)
    cpdef bool keyDown(self)
    cpdef bool keyUp(self)

    @staticmethod
    cdef KeyboardEvent wrap(const cKeyboardEvent& data)

cdef class MouseEvent:
    cdef shared_ptr[cMouseEvent] _ptr
    cdef inline cMouseEvent* ptr(self)

    @staticmethod
    cdef MouseEvent wrap(const cMouseEvent& data)

cdef class PointPickingEvent:
    cdef shared_ptr[cPointPickingEvent] _ptr
    cdef inline cPointPickingEvent* ptr(self)

    @staticmethod
    cdef PointPickingEvent wrap(const cPointPickingEvent& data)

cdef class AreaPickingEvent:
    cdef shared_ptr[cAreaPickingEvent] _ptr
    cdef inline cAreaPickingEvent* ptr(self)

    @staticmethod
    cdef AreaPickingEvent wrap(const cAreaPickingEvent& data)

cdef class VisualizerInteractorStyle:
    cdef vtkSmartPointer[PCLVisualizerInteractorStyle] _ptr
    cdef inline PCLVisualizerInteractorStyle* ptr(self)

    cpdef void saveCameraParameters(self, str file)
    cpdef void loadCameraParameters(self, str file)

cdef class Visualizer:
    cdef shared_ptr[PCLVisualizer] _ptr
    cdef object __weakref__
    cdef inline PCLVisualizer* ptr(self)

    cpdef void setFullScreen(self, bool mode) except*
    cpdef void setWindowName(self, str name) except*
    cpdef void setWindowBorders(self, bool mode) except*
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewport=*) except*

    # TODO: return wrapped connection object
    cpdef void registerKeyboardCallback(self, callback) except*
    cpdef void registerMouseCallback(self, callback) except*
    cpdef void registerPointPickingCallback(self, callback) except*
    cpdef void registerAreaPickingCallback(self, callback) except*

    cpdef void spin(self) except*
    cpdef void spinOnce(self, int time=*, bool force_redraw=*) except*

    cpdef void addCoordinateSystem(self, double scale=*, float x=*, float y=*, float z=*, np.ndarray t=*, int viewport=*) except*
    cpdef void removeCoordinateSystem(self, int viewport=*) except*

    cpdef void removePointCloud(self, str id=*, int viewport=*) except*
    cpdef void removePolygonMesh(self, str id=*, int viewport=*) except*
    cpdef void removeShape(self, str id=*, int viewport=*) except*
    cpdef void removeText3D(self, str id=*, int viewport=*) except*
    cpdef void removeAllPointClouds(self, int viewport=*) except*
    cpdef void removeAllShapes(self, int viewport=*) except*

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=*, color=*, str id=*, int viewport=*) except*
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=*, color=*, str id=*) except*
    cpdef void addText3D(self, str text, position, double text_scale=*, color=*, str id=*, int viewport=*) except*
    cpdef void addPointCloudNormals(self, PointCloud cloud, PointCloud normals=*, int level=*, float scale=*, str id=*, int viewport=*) except*
    cpdef void addPointCloudPrincipalCurvatures(self, PointCloud cloud, PointCloud normals, PointCloud pcs, int level=*, float scale=*, str id=*, int viewport=*) except*
    cpdef void addPointCloudIntensityGradients(self, PointCloud cloud, PointCloud gradients, int level=*, double scale=*, str id=*, int viewport=*) except*

    cpdef void addCorrespondences(self, PointCloud source_points, PointCloud target_points, correspondences, str id=*, int viewport=*) except*
    cpdef void removeCorrespondences (self, str id=*, int viewport=*) except*

    cpdef void updateShapePose(self, str id, np.ndarray pose) except*
    cpdef void updatePointCloudPose(self, str id, np.ndarray pose) except*

    cpdef bool wasStopped(self) except*
    cpdef void resetStoppedFlag(self) except*
    cpdef void close(self) except*
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax) except*
    cpdef void createViewPortCamera(self, int viewport) except*

    cpdef void addPointCloud(self, PointCloud cloud, color=*, str field=*, color_handler=*, bint static_mapping=*, str id=*, int viewport=*) except*
    cpdef void updatePointCloud(self, PointCloud cloud, str id=*) except*

    cpdef void addLine(self, p1, p2, color=*, str id=*, int viewport=*) except*
    cpdef void addArrow(self, p1, p2, line_color=*, text_color=*, bool display_length=*, str id=*, int viewport=*) except*
    cpdef void addSphere(self, center, double radius, color=*, str id=*, int viewport=*) except*
    cpdef void updateSphere(self, center, double radius, color=*, str id=*) except*
    cpdef void addCylinder(self, point_on_axis, axis_direction, double radius, str id=*, int viewport=*) except*
    cpdef void addPlane(self, coeffs, str id=*, int viewport=*) except*
    cpdef void addCircle(self, center, double radius, str id=*, int viewport=*) except*
    cpdef void addCube(self, translation, rotation, double width, double height, double depth, str id=*, int viewport=*) except*

    cpdef void setShapeRenderingProperties(self, property, value, str id, int viewport=*) except*
    cpdef void setPointCloudRenderingProperties(self, property, value, str id, int viewport=*) except*
    cpdef void setRepresentationToSurfaceForAllActors(self) except*
    cpdef void setRepresentationToPointsForAllActors(self) except*
    cpdef void setRepresentationToWireframeForAllActors(self) except*
    
    cpdef void initCameraParameters(self) except*
    cpdef void getCameraParameters(self, list argv) except*
    cpdef bool cameraParamsSet(self) except*
    cpdef void updateCamera(self) except*
    cpdef void resetCamera(self) except*
    cpdef void resetCameraViewpoint(self, str id=*) except*
    cpdef void setCameraPosition(self, position, view_up, focal_point=*, int viewport=*) except*
    cpdef void setCameraFieldOfView(self, double fovy, int viewport=*) except*
    cpdef void setCameraClipDistances(self, double near, double far, int viewport=*) except*

    cpdef void setPosition(self, int x, int y) except*
    cpdef void setSize(self, int xw, int yw) except*
    cpdef void setUseVbos(self, bool use_vbox) except*
    cpdef void createInteractor(self) except*
    cpdef void saveScreenshot(self, str file) except*
    cpdef void setShowFPS(self, bool show_fps) except*

    cpdef VisualizerInteractorStyle getInteractorStyle(self)
