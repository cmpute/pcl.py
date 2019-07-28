# Features
- [ ] Add unit tests and tests
- [ ] Implement some official test codes
- [x] Add conversion between numpy array and Eigen objects
- [ ] Migrate original implementations of pypcl
- [ ] Fix TODOs, XXXs and FIXMEs in the code
- [ ] Support organized point cloud
- [ ] Support PCLImage, PolygonMesh, RangeImage
- [ ] Full support customized point cloud fields
- [x] Instantiate template classes to given point-type (e.g. PointXYZ, PointXYZRGB), to implement visualizers and various template modules
- [ ] Add docstring and documentations
- [ ] Deal with exceptions by `except +`

# Bugs
- Access field immediately after initialization will return different data: `d=pcl.PointCloud([1,2,3]), d.xyz` vs `pcl.PointCloud([1,2,3]).xyz`
