# Features
- [x] Add conversion between numpy array and Eigen objects
- [x] Full support customized point cloud fields
- [x] Instantiate template classes to given point-type (e.g. PointXYZ, PointXYZRGB), to implement visualizers and various template modules
- [x] Deal with exceptions by `except +`
- [ ] Add more unit tests and tests (e.g for ROS support)
- [ ] Implement some official test codes
- [ ] Migrate original implementations of pypcl
- [ ] Fix TODOs, XXXs and FIXMEs in the code
- [ ] Support organized point cloud
- [ ] Support PCLImage, PolygonMesh, RangeImage
- [ ] Add docstring and documentations

# Modules to be wrapped
- pcl::PCLHeader
- pcl::Correspondence
- pcl::MsgFieldMap
- boost::connection
- pcl::PCLImage
- pcl::PolygonMesh
- pcl::RangeImage
