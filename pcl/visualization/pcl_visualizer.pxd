from libcpp cimport bool
from libcpp.string cimport string
from pcl._boost.signals2 cimport connection
from pcl._boost.smart_ptr cimport shared_ptr
from pcl._eigen cimport Affine3f
from pcl.common.point_types cimport PointXYZ, Normal, PrincipalCurvatures
from pcl.common.point_cloud cimport PointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.visualization.area_picking_event cimport AreaPickingEvent
from pcl.visualization.keyboard_event cimport KeyboardEvent
from pcl.visualization.mouse_event cimport MouseEvent
from pcl.visualization.point_picking_event cimport PointPickingEvent
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2

cdef extern from "pcl/visualization/pcl_visualizer.h" namespace "pcl::visualization":
    cdef cppclass PCLVisualizer:
        # PCLVisualizer (const std::string &name = "", const bool create_interactor = true)
        PCLVisualizer(const string &name, bool create_interactor)
        # PCLVisualizer (int &argc, char **argv, const std::string &name = "",    PCLVisualizerInteractorStyle* style = PCLVisualizerInteractorStyle::New (), const bool create_interactor = true)

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
        bool removeCoordinateSystem(int viewpoint)

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
        void setBackgroundColor(const double &r, const double &g, const double &b, int viewpoint)

        # bool addText (const std::string &text, int xpos, int ypos, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, const string &id, int viewpoint)
        # bool addText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, double r, double g, double b, const string &id, int viewpoint)
        # bool addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id = "", int viewport = 0);
        bool addText(const string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const string &id, int viewpoint)
        # bool updateText (const std::string &text, int xpos, int ypos, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, const string &id)
        # bool updateText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, double r, double g, double b,const string &id)
        # bool updateText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id = "");
        bool updateText(const string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const string &id)

        bool updateShapePose(const string &id, const Affine3f &pose)
        bool updatePointCloudPose(const string &id, const Affine3f &pose)

        # template <typename PointT> bool addText3D (const std::string &text, const PointT &position, double textScale = 1.0, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "", int viewport = 0);
        bool addText3D[PointT](const string &text, const PointT &position, double textScale, double r, double g, double b, const string &id, int viewpoint)

        # XXX: template <typename PointNT> bool addPointCloudNormals (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, int level = 100, float scale = 0.02f, const std::string &id = "cloud", int viewport = 0);

        # XXX: template <typename PointT, typename PointNT> bool addPointCloudNormals (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const typename pcl::PointCloud<PointNT>::ConstPtr &normals, int level = 100, float scale = 0.02f, const std::string &id = "cloud", int viewport = 0);

        # bool addPointCloudPrincipalCurvatures (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr &pcs, int level = 100, float scale = 1.0f, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudPrincipalCurvatures(const shared_ptr[const PointCloud[PointXYZ]] &cloud, shared_ptr[const PointCloud[Normal]] &normals, const shared_ptr[const PointCloud[PrincipalCurvatures]] &pcs, int level, float scale,const string &id, int viewport);

        # template <typename PointT, typename GradientT> bool addPointCloudIntensityGradients (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const typename pcl::PointCloud<GradientT>::ConstPtr &g const std::string &id = "cloud", int viewport = 0);
        bool addPointCloudIntensityGradients [PointT, GradientT](const shared_ptr[const PointCloud[PointT]] &cloud, const shared_ptr[const PointCloud[GradientT]] &gradients, int level, double scale, const string &id, int viewport = 0);

        # template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "cloud", int viewport = 0);
        bool addPointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const string &id, int viewport)
        # template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "cloud");
        bool updatePointCloud[PointT] (const shared_ptr[const PointCloud[PointT]] &cloud, const string &id)
        # XXX: template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudGeometryHandler<PointT> &geometry_handler, const std::string &id = "cloud");
        # XXX: template <typename PointT> bool updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudColorHandler<PointT> &color_handler, const std::string &id = "cloud");
        # XXX: template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudGeometryHandler<PointT> &geometry_handler, const std::string &id = "cloud", int viewport = 0);
        # XXX: template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const std::string &id = "cloud", int viewport = 0);
        # XXX: template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const PointCloudColorHandler<PointT> &color_handler, const std::string &id = "cloud", int viewport = 0);
        # XXX: template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const ColorHandlerConstPtr &color_handler, const std::string &id = "cloud", int viewport = 0);
        # XXX: template <typename PointT> bool addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const ColorHandlerConstPtr &color_handler, const std::string &id = "cloud", int viewport = 0);

        # XXX: bool addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud, const GeometryHandlerConstPtr &geometry_handler, const ColorHandlerConstPtr &color_handler, const Eigen::Vector4f& sensor_origin, const Eigen::Quaternion<float>& sensor_orientation, const std::string &id = "cloud", int viewport = 0);
        
        # TODO: continue at line 707
