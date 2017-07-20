# Roadmap

## Original Implementations
- Implement Polygon Related classes (Vertices, Mesh, Polygon, etc.)
- io with vtk files
- visualization opengl(vtk or [higher package](http://old.sebug.net/paper/books/scipydoc/mlab_and_mayavi.html)?)/matplotlib
- io from .ply/.stl/other mesh files (refer to PyMesh)
- implement search interfaces (prior: nmslib, scikit, octree; alter: pyflann3, falconn)
- pcdenoise | pcdownsample | pcfitplane | pcmerge | pcplayer | pcregrigid | pcshow | pctransform | pcwrite | planeModel (MATLAB functions)
- interact with ros (sensor_msg package) / vrep / airsim ?

## Additional Implementations
- 2D Range Tree with fascading to solve kNN or ANN
- Pointcloud crop/cut/select tool
- implement .show() for PointCloud to display
- implement additional features in [Geometry++](http://threepark.net/geometryplusplus/feature)

## Code qualities and performance
- add docs generating and tidy up documentation
- fix coverall searching and add pip install test in travis
- add enough debug logging info
- optimize point cloud operations with the help of [numpy.lib.recfunctions](https://github.com/numpy/numpy/blob/master/numpy/lib/recfunctions.py) codes

## Visualization
Should refer to [ParaView](https://www.paraview.org/) / ~~[3D Slicer](https://www.slicer.org/)~~ / ~~[ImageVis3D](http://www.sci.utah.edu/software/imagevis3d.html)~~ / [CloudCompare](http://www.danielgm.net/cc/)

Annotation tools may be provided with following functions:
- Four Models: Selected Point Cloud; Selected Body(Ploygon or others); Boolean Operations; Tags
- Three Base objects: Point -> Divider(Plane/Sphere/etc) -> Spatial Region
- Points can be selected by Click/View Box/Spatial Region
- Divider Plane can be defined by Parameters/Three points/Default with after-manufacturing.
- Divider/Selected Body can be translated/rotated by view axis or translated/rotated by coordinate axis.

## Issues
- if XYZ is not 4-byte float (e.g 8-byte), the viewer will set to wrong scale
- `PointCloud([1,2,3], ['x', 'y', 'z'])` will cause single element problems, while `PointCloud([(1,2,3)], ['x', 'y', 'z'])` won't
- precision problem in NormalEstimation