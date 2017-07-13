# Roadmap

## Original Implementations
- add coverage and pip install test in travis
- implement search interfaces (prior: bruteforce, nmslib, scikit, octree; alter: pyflann3, falconn)
- io from .ply file
- visualization opengl(vtk?)/matplotlib
- pcdenoise | pcdownsample | pcfitplane | pcmerge | pcplayer | pcregrigid | pcshow | pctransform | pcwrite | planeModel (MATLAB functions)
- interact with ros (sensor_msg package) / vrep / airsim ?
- add enough debug logging info

## Additional Implementations
- 2D Range Tree with fascading to solve kNN or ANN
- Pointcloud crop/cut/select tool
- implement .show() for PointCloud to display

## Issues 
- if XYZ is not 4-byte float (e.g 8-byte), the viewer will set to wrong scale
- `PointCloud([1,2,3], ['x', 'y', 'z'])` will cause single element problems, while `PointCloud([(1,2,3)], ['x', 'y', 'z'])` won't