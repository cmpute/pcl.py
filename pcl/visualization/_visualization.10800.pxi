from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandlerRGBAField_PCLPointCloud2, PointCloudColorHandlerLabelField_PCLPointCloud2

cdef inline void addPointCloud_LabelField(PCLVisualizer* vis_ptr, PointCloud cloud, shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler, bint static_mapping, str id, int viewport) except*:
    cdef shared_ptr[PointCloudColorHandlerLabelField_PCLPointCloud2] label_handler = \
        make_shared[PointCloudColorHandlerLabelField_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, static_mapping)
    _ensure_true(vis_ptr.addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
        <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
        <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>label_handler,
        cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
        "addPointCloud")

cdef inline void addPointCloud_RGBAField(PCLVisualizer* vis_ptr, PointCloud cloud, shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler, str id, int viewport) except*:
    cdef shared_ptr[PointCloudColorHandlerRGBAField_PCLPointCloud2] rgba_handler = \
        make_shared[PointCloudColorHandlerRGBAField_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
    _ensure_true(vis_ptr.addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
        <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>xyz_handler,
        <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>rgba_handler,
        cloud._origin, cloud._orientation, id.encode('ascii'), viewport),
        "addPointCloud")
