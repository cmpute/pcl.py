cdef inline void addPointCloud_LabelField(PCLVisualizer* vis_ptr, PointCloud cloud, shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler, bint static_mapping, str id, int viewport) except*:
    raise NotImplementedError("Only supported in PCL 1.8 and higher")

cdef inline void addPointCloud_RGBAField(PCLVisualizer* vis_ptr, PointCloud cloud, shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] xyz_handler, str id, int viewport) except*:
    raise NotImplementedError("Only supported in PCL 1.8 and higher")
