# PyPCL [![Build Status](https://travis-ci.org/cmpute/pypcl.svg?branch=master)](https://travis-ci.org/cmpute/pypcl) [![Coverage Status](https://coveralls.io/repos/github/cmpute/pypcl/badge.svg)](https://coveralls.io/github/cmpute/pypcl)

Re-implementation in pure python of Point Cloud Library (PCL)

## Principles
Implement a easy-using python library to process point cloud, combined with scipy and numpy. It can helps you get rid of the problems of linking various C++ libraries and the long waiting compiling process . The interfaces of PCL are also cleaned and simplified for handy using.

Nevertheless, this library focuses on simplicity, readability and accessibility, and speed performance may be sacrificed when developing. Code optimizing is not conducted generally, and compared to MATLAB and C++, the library can be a bit slower in some cases.

------------------------

## Compatibility
- Written in Python 3
- Tested on Python 3.5, Numpy>=1.10 (Temporarily)

------------------------

## Specifics
- PCL Point Cloud is implemented, while PCL Image/Video not, only interfaces to opencv are exposed
- Polygon/Mesh is only partly implemented so that the data can interact with other packages

------------------------

## Notes
The implementations may not be exactly the same as pcl (since this library will take advantages of many other packages in python), however, interfaces are expected to remain the same. Major differences are listed below:
- Naming style of functions is changed from mixedCase (optimizeCoefficients) to lower_case_with_underscore (optimize_coefficients).
- 'PCL' in class names are removed
- Functions named getXXX and setXXX are turned into properties xxx.

The library is under heavy construction, thus do not use it in productive codes. However, playing with it is totally welcome now, and it will be great to receive your issues, suggestions and pull requests!~
