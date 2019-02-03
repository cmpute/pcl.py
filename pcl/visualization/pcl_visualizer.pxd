from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from pcl._boost cimport connection, shared_ptr
from pcl._eigen cimport Affine3f, Matrix3f, Matrix4f, Vector3f, Vector4f, Quaternionf
from pcl.common.ModelCoefficients cimport ModelCoefficients
from pcl.common.point_types cimport PointXYZ, PointXYZRGB, PointXYZRGBA, Normal, PrincipalCurvatures
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.common.PolygonMesh cimport PolygonMesh
from pcl.common.Vertices cimport Vertices
from pcl.visualization.area_picking_event cimport AreaPickingEvent
from pcl.visualization.keyboard_event cimport KeyboardEvent
from pcl.visualization.mouse_event cimport MouseEvent
from pcl.visualization.point_picking_event cimport PointPickingEvent
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler, PointCloudColorHandler_PCLPointCloud2
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler, PointCloudGeometryHandler_PCLPointCloud2

# XXX: optional args in extern functions can sometimes generate errors
cdef extern from "pcl/visualization/pcl_visualizer.h" namespace "pcl::visualization":
    cdef cppclass PCLVisualizer:
        PCLVisualizer()
        # PCLVisualizer (const std::string &name = "", const bool create_interactor = true)
        PCLVisualizer(const string &name, bool create_interactor)
        # XXX: PCLVisualizer (int &argc, char **argv, const std::string &name = "", PCLVisualizerInteractorStyle* style = PCLVisualizerInteractorStyle::New (), const bool create_interactor = true)

        void setFullScreen(bool mode)
        void setWindowName(const string &name)
        void setWindowBorders(bool mode)
        
        # boost::signals2::connection registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL);
        connection registerKeyboardCallback(void (*callback)(const KeyboardEvent&, void*), void* cookie)
        # boost::signals2::connection registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*), void* cookie = NULL);
        connection registerMouseCallback(void (*callback) (const MouseEvent&, void*), void* cookie)
        # boost::signals2::connection registerPointPickingCallback (void (*callback) (const pcl::visualization::PointPickingEvent&, void*), void* cookie = NULL);
        connection registerPointPickingCallback(void (*callback) (const PointPickingEvent&, void*), void* cookie)
        # boost::signals2::connection registerAreaPickingCallback (void (*callback) (const pcl::visualization::AreaPickingEvent&, void*), void* cookie = NULL);
        connection registerAreaPickingCallback(void (*callback) (const AreaPickingEvent&, void*), void* cookie)

        void spin()
        # void spinOnce (int time = 1, bool force_redraw = false);
        void spinOnce(int time, bool force_redraw)

        # XXX: void addOrientationMarkerWidgetAxes(vtkRenderWindowInteractor* interactor)
        void removeOrientationMarkerWidgetAxes ()

        # void addCoordinateSystem (double scale = 1.0, int viewport = 0);
        void addCoordinateSystem(double scale, int viewport)
        # void addCoordinateSystem (double scale, float x, float y, float z, int viewport = 0);
        void addCoordinateSystem(double scale, float x, float y, float z, int viewport)
        # void addCoordinateSystem (double scale, const Eigen::Affine3f& t, int viewport = 0);
        void addCoordinateSystem(double scale, const Affine3f &t, int viewport)
        # bool removeCoordinateSystem (int viewport = 0);
        bool removeCoordinateSystem(int viewport)

        # bool removePointCloud (const std::string &id = "cloud", int viewport = 0);
        bool removePointCloud(const string &id, int viewport)
        # inline bool removePolygonMesh (const std::string &id = "polygon", int viewport = 0)
        bool removePolygonMesh(const string &id, int viewport)
        # bool removeShape (const std::string &id = "cloud", int viewport = 0);
        bool removeShape(const string &id, int viewport)
        # bool removeText3D (const std::string &id = "cloud", int viewport = 0);
        bool removeText3D(const string &id, int viewport)
        # bool removeAllPointClouds (int viewport = 0);
        bool removeAllPointClouds(int viewport)
        # bool removeAllShapes (int viewport = 0);
        bool removeAllShapes(int viewport)
        
        # void setBackgroundColor (const double &r, const double &g, const double &b, int viewport = 0);
        void setBackgroundColor(const double &r, const double &g, const double &b, int viewport)

        # bool addText (const std::string &text, int xpos, int ypos, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, const string &id, int viewport)
        # bool addText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, double r, double g, double b, const string &id, int viewport)
        # bool addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const string &id, int viewport)
        # bool updateText (const std::string &text, int xpos, int ypos, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, const string &id)
        # bool updateText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, double r, double g, double b, const string &id)
        # bool updateText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const string &id)

        bool updateShapePose(const string &id, const Affine3f &pose)
        bool updatePointCloudPose(const string &id, const Affine3f &pose)

        # template <typename PointT> bool addText3D (const std::string &text, const PointT &position, double textScale = 1.0, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "", int viewport = 0);
        bool addText3D[PointT](const string &text, const PointT &position, double textScale, double r, double g, double b, const string &id, int viewport)
        # template <typename PointNT> bool addPointCloudNormals (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, int level = 100, float scale = 0.02f, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudNormals[PointNT](const shared_ptr[PointCloud[PointNT]] &cloud, int level, float scale, const string &id, int viewport)
        # template <typename PointT, typename PointNT> bool addPointCloudNormals (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const typename pcl::PointCloud<PointNT>::ConstPtr &normals, int level = 100, float scale = 0.02f, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudNormals[PointT, PointNT](const shared_ptr[PointCloud[PointT]] &cloud, const shared_ptr[PointCloud[PointNT]] &normals, int level, float scale, const string &id, int viewport)
        # bool addPointCloudPrincipalCurvatures (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr &pcs, int level = 100, float scale = 1.0f, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudPrincipalCurvatures(const shared_ptr[const PointCloud[PointXYZ]] &cloud, shared_ptr[const PointCloud[Normal]] &normals, const shared_ptr[const PointCloud[PrincipalCurvatures]] &pcs, int level, float scale, const string &id, int viewport);

        # template <typename PointT, typename GradientT> bool addPointCloudIntensityGradients (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const typename pcl::PointCloud<GradientT>::ConstPtr &radients, int level = 100, double scale = 1e-6, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudIntensityGradients [PointT, GradientT](const shared_ptr[const PointCloud[PointT]] &cloud, const shared_ptr[const PointCloud[GradientT]] &gradients, int level, double scale, const string &id, int viewport = 0);

        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const string &id, int viewport)
        # template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "cloud");
        bool updatePointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const string &id)
        # template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudGeometryHandler<PointT> &geometry_handler, const std::string &id = "cloud");
        bool updatePointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const PointCloudGeometryHandler[PointT] &geometry_handler, const string &id)
        # template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudColorHandler<PointT> &color_handler, const std::string &id = "cloud");
        bool updatePointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const PointCloudColorHandler[PointT] &color_handler, const string &id)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudGeometryHandler<PointT> &geometry_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const PointCloudGeometryHandler[PointT] &geometry_handler, const string &id, int viewport)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2] &geometry_handler, const string &id, int viewport)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudColorHandler<PointT> &color_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const PointCloudColorHandler[PointT] &color_handler, const string &id, int viewport)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const ColorHandlerConstPtr &color_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const shared_ptr[const PointCloudColorHandler_PCLPointCloud2] &color_handler, const string &id, int viewport)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const ColorHandlerConstPtr &color_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2] &geometry_handler, const shared_ptr[const PointCloudColorHandler_PCLPointCloud2] &color_handler, const string &id, int viewport)

        # bool addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const ColorHandlerConstPtr &color_handler, const Eigen::Vector4f& sensor_origin, const Eigen::Quaternion<float>& sensor_orientation, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud (const PCLPointCloud2ConstPtr &cloud, const shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2] &geometry_handler, const shared_ptr[const PointCloudColorHandler_PCLPointCloud2] &color_handler, const Vector4f &sensor_origin, const Quaternionf &sensor_orientation, const string &id, int viewport)
        # bool addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const Eigen::Vector4f& sensor_origin, const Eigen::Quaternion<float>& sensor_orientation, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud (const PCLPointCloud2ConstPtr &cloud, const shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2] &geometry_handler, const Vector4f &sensor_origin, const Quaternionf &sensor_orientation, const string &id, int viewport)
        # bool addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud, const ColorHandlerConstPtr &color_handler, const Eigen::Vector4f& sensor_origin, const Eigen::Quaternion<float>& sensor_orientation, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud (const PCLPointCloud2ConstPtr &cloud, const shared_ptr[const PointCloudColorHandler_PCLPointCloud2] &color_handler, const Vector4f &sensor_origin, const Quaternionf &sensor_orientation, const string &id, int viewport)
        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudColorHandler<PointT> &color_handler, const PointCloudGeometryHandler<PointT> &geometry_handler, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const PointCloudColorHandler[PointT] &color_handler, const PointCloudGeometryHandler[PointT] &geometry_handler, const string &id, int viewport)

        # inline bool addPointCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const std::string &id = "cloud", int viewport = 0)
        bool addPointCloud (const shared_ptr[const PointCloud[PointXYZ]] &cloud, const string &id, int viewport)
        # inline bool addPointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, const std::string &id = "cloud", int viewport = 0)
        bool addPointCloud (const shared_ptr[const PointCloud[PointXYZRGB]] &cloud, const string &id, int viewport)
        # inline bool addPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::string &id = "cloud", int viewport = 0)
        bool addPointCloud (const shared_ptr[const PointCloud[PointXYZRGBA]] &cloud, const string &id, int viewport)
        # inline bool updatePointCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const std::string &id = "cloud")
        bool updatePointCloud_XYZ "updatePointCloud<pcl::PointXYZ>" (const shared_ptr[const PointCloud[PointXYZ]] &cloud, const string &id)
        # inline bool updatePointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, const std::string &id = "cloud")
        bool updatePointCloud_XYZRGB "updatePointCloud<pcl::PointXYZRGB>" (const shared_ptr[const PointCloud[PointXYZRGB]] &cloud, const string &id)
        # inline bool updatePointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::string &id = "cloud")
        bool updatePointCloud_XYZRGBA "updatePointCloud<pcl::PointXYZRGBA>" (const shared_ptr[const PointCloud[PointXYZRGBA]] &cloud, const string &id)

        # bool addPolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polygon", int viewport = 0);
        bool addPolygonMesh(const PolygonMesh &polymesh, const string &id, int viewport)
        # template <typename PointT> bool addPolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<pcl::Vertices> &vertices, const std::string &id = "polygon", int viewport = 0);
        bool addPolygonMesh[PointT](const shared_ptr[const PointCloud[PointT]] &cloud, const vector[Vertices] &vertices, const string &id, int viewport)
        # template <typename PointT> bool updatePolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<pcl::Vertices> &vertices, const std::string &id = "polygon");
        bool updatePolygonMesh[PointT](const shared_ptr[const PointCloud[PointT]] &cloud, const vector[Vertices] &vertices, const string &id)
        # bool updatePolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polygon");
        bool updatePolygonMesh (const PolygonMesh &polymesh, const string &id)
        # bool addPolylineFromPolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polyline", int viewport = 0);
        bool addPolylineFromPolygonMesh (const PolygonMesh &polymesh, const string 
        &id, int viewport)

        # template <typename PointT> bool addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points, const typename pcl::PointCloud<PointT>::ConstPtr &target_points, const std::vector<int> & correspondences, const std::string &id = "correspondences", int viewport = 0);
        bool addCorrespondences[PointT](const shared_ptr[const PointCloud[PointT]] &source_points, const shared_ptr[const PointCloud[PointT]] &target_points, const vector[int] &correspondences, const string &id, int viewport)
        # XXX: template <typename PointT> bool addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points, const typename pcl::PointCloud<PointT>::ConstPtr &target_points, const pcl::Correspondences &correspondences, int nth, const std::string &id = "correspondences", int viewport = 0);
        # XXX: template <typename PointT> bool addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points, const typename pcl::PointCloud<PointT>::ConstPtr &target_points, const pcl::Correspondences &correspondences, const std::string &id = "correspondences", int viewport = 0)
        # XXX: template <typename PointT> bool updateCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points, const typename pcl::PointCloud<PointT>::ConstPtr &target_points, const pcl::Correspondences &correspondences, int nth, const std::string &id = "correspondences");
        # void removeCorrespondences (const std::string &id = "correspondences", int viewport = 0);
        void removeCorrespondences (const string &id, int viewport)

        # int getColorHandlerIndex (const std::string &id);
        int getColorHandlerIndex (const string &id)
        # int getGeometryHandlerIndex (const std::string &id);
        int getGeometryHandlerIndex (const string &id)
        # bool updateColorHandlerIndex (const std::string &id, int index);
        bool updateColorHandlerIndex (const string &id, int index)
        # bool setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const std::string &id = "cloud", int viewport = 0);

        bool setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const string &id, int viewport)
        # bool setPointCloudRenderingProperties (int property, double value, const std::string &id = "cloud", int viewport = 0);
        bool setPointCloudRenderingProperties (int property, double value, const string &id, int viewport)
        # bool getPointCloudRenderingProperties (int property, double &value, const std::string &id = "cloud");
        bool getPointCloudRenderingProperties (int property, double &value, const string &id)
        # bool setPointCloudSelected (const bool selected, const std::string &id = "cloud" );
        bool setPointCloudSelected (bool selected, const string &id);
        # bool setShapeRenderingProperties (int property, double value, const std::string &id, int viewport = 0);
        bool setShapeRenderingProperties (int property, double value, const string &id, int viewport)

        bool wasStopped()
        void resetStoppedFlag()
        void close()
        void createViewPort(double xmin, double ymin, double xmax, double ymax, int &viewport)
        void createViewPortCamera(int viewport)

        # template <typename PointT> bool addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, double r, double g, double b, const std::string &id = "polygon", int viewport = 0);
        bool addPolygon[PointT](const shared_ptr[const PointCloud[PointT]] &cloud, double r, double g, double b, const string &id, int viewport)
        # template <typename PointT> bool addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "polygon", int viewport = 0);
        bool addPolygon[PointT](const shared_ptr[const PointCloud[PointT]] &cloud, const string &id, int viewport)
        # XXX: template <typename PointT> bool addPolygon (const pcl::PlanarPolygon<PointT> &polygon, double r, double g, double b, const std::string &id = "polygon", int viewport = 0);

        # template <typename P1, typename P2> bool addLine (const P1 &pt1, const P2 &pt2, const std::string &id = "line", int viewport = 0);
        bool addLine[P1, P2](const P1 &pt1, const P2 &pt2, const string &id, int viewport)
        # template <typename P1, typename P2> bool addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id = "line", int viewport = 0);
        bool addLine[P1, P2](const P1 &pt1, const P2 &pt2, double r, double g, double b, const string &id, int viewport)
        # template <typename P1, typename P2> bool addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id = "arrow", int viewport = 0);
        bool addArrow[P1, P2](const P1 &pt1, const P2 &pt2, double r, double g, double b, const string &id, int viewport)
        # template <typename P1, typename P2> bool addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length, const std::string &id = "arrow", int viewport = 0);
        bool addArrow[P1, P2](const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length, const string &id, int viewport)
        # template <typename P1, typename P2> bool addArrow (const P1 &pt1, const P2 &pt2, double r_line, double g_line, double b_line, double r_text, double g_text, double b_text, const std::string &id = "arrow", int viewport = 0);
        bool addArrow[P1, P2](const P1 &pt1, const P2 &pt2, double r_line, double g_line, double b_line, double r_text, double g_text, double b_text, const string &id, int viewport)
        # template <typename PointT> bool addSphere (const PointT &center, double radius, const std::string &id = "sphere", int viewport = 0);
        bool addSphere[PointT](const PointT &center, double radius, const string &id, int viewport)
        # template <typename PointT> bool addSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id = "sphere", int viewport = 0);
        bool addSphere[PointT](const PointT &center, double radius, double r, double g, double b, const string &id, int viewport)
        # template <typename PointT> bool updateSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id = "sphere");
        bool updateSphere[PointT](const PointT &center, double radius, double r, double g, double b, const string &id)

        # XXX: bool addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, const std::string & id = "PolyData", int viewport = 0);
        # XXX: bool addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PolyData", int viewport = 0);
        # bool addModelFromPLYFile (const std::string &filename, const std::string &id = "PLYModel", int viewport = 0);
        bool addModelFromPLYFile(const string &filename, const string &id, int viewport)
        # XXX: bool addModelFromPLYFile (const std::string &filename, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PLYModel", int viewport = 0);

        # bool addCylinder (const pcl::ModelCoefficients &coefficients, const std::string &id = "cylinder", int viewport = 0);
        bool addCylinder(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addSphere (const pcl::ModelCoefficients &coefficients, const std::string &id = "sphere", int viewport = 0);
        bool addSphere(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addLine (const pcl::ModelCoefficients &coefficients, const std::string &id = "line", int viewport = 0);
        bool addLine(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addPlane (const pcl::ModelCoefficients &coefficients, const std::string &id = "plane", int viewport = 0);
        bool addPlane(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id = "plane", int viewport = 0);
        bool addPlane(const ModelCoefficients &coefficients, double x, double y, double z, string &id, int viewport)
        # bool addCircle (const pcl::ModelCoefficients &coefficients, const std::string &id = "circle", int viewport = 0);
        bool addPlane(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addCone (const pcl::ModelCoefficients &coefficients, const std::string &id = "cone", int viewport = 0);
        bool addCone(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addCube (const pcl::ModelCoefficients &coefficients, const std::string &id = "cube", int viewport = 0);
        bool addCube(const ModelCoefficients &coefficients, const string &id, int viewport)
        # bool addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double width, double height, double depth, const std::string &id = "cube", int viewport = 0);
        bool addCube(const Vector3f &translation, const Quaternionf &rotation, double width, double height, double depth, const string &id, int viewport)
        # bool addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);
        bool addCube(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, double r, double g, double b, const string &id, int viewport)

        void setRepresentationToSurfaceForAllActors()
        void setRepresentationToPointsForAllActors()
        void setRepresentationToWireframeForAllActors()
        void setShowFPS (bool show_fps)

        void renderView (int xres, int yres, shared_ptr[PointCloud[PointXYZ]] & cloud)
        # XXX: void renderViewTesselatedSphere (int xres, int yres, pcl::PointCloud<pcl::PointXYZ>::CloudVectorType & cloud, std::vector<Eigen::Matrix4f,Eigen::aligned_allocator< Eigen::Matrix4f > > & poses, std::vector<float> & enthropies, int tesselation_level, float view_angle = 45, float radius_sphere = 1, bool use_vertices = true);
        
        void initCameraParameters ()
        bool getCameraParameters (int argc, char **argv)
        bool cameraParamsSet ()
        void updateCamera ()
        void resetCamera ()
        # void resetCameraviewport (const std::string &id = "cloud");
        void resetCameraviewport (const string &id)
        # void setCameraPosition (double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport = 0);
        void setCameraPosition (double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport)
        # void setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport = 0);
        void setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport)
        # void setCameraParameters (const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics, int viewport = 0);
        void setCameraParameters (const Matrix3f &intrinsics, const Matrix4f &extrinsics, int viewport)
        # XXX: void setCameraParameters (const Camera &camera, int viewport = 0);
        # void setCameraClipDistances (double near, double far, int viewport = 0);
        void setCameraClipDistances (double near, double far, int viewport)
        void setCameraFieldOfView (double fovy, int viewport)
        # XXX: void getCameras (std::vector<Camera>& cameras);
        # Eigen::Affine3f getViewerPose (int viewport = 0);
        Affine3f getViewerPose (int viewport)
        void saveScreenshot (const string &file)

        # XXX: vtkSmartPointer<vtkRenderWindow> getRenderWindow ()
        # XXX: vtkSmartPointer<vtkRendererCollection> getRendererCollection ()
        # XXX: CloudActorMapPtr getCloudActorMap ()
        
        void setPosition (int x, int y)
        void setSize (int xw, int yw)
        void setUseVbos (bool use_vbos)
        void createInteractor ()

        # XXX: void setupInteractor (vtkRenderWindowInteractor *iren, vtkRenderWindow *win);
        # XXX: void setupInteractor (vtkRenderWindowInteractor *iren, vtkRenderWindow *win, vtkInteractorStyle *style);
        # XXX: inline vtkSmartPointer<PCLVisualizerInteractorStyle> getInteractorStyle ()

ctypedef shared_ptr[PCLVisualizer] PCLVisualizerPtr
ctypedef shared_ptr[const PCLVisualizer] PCLVisualizerConstPtr
