# PyPCL [![Build Status](https://travis-ci.org/cmpute/pypcl.svg?branch=master)](https://travis-ci.org/cmpute/pypcl) [![Coverage Status](https://coveralls.io/repos/github/cmpute/pypcl/badge.svg)](https://coveralls.io/github/cmpute/pypcl)

Cython bindings of Point Cloud Library (PCL)

## Principles
Implement a easy-using cython library to process point cloud, combined with scipy and numpy. This library wraps `PCLPointCloud2` class into python and users can pass data from numpy to `PointCloud<PointT>` easily with this library and headers.

Nevertheless, this library focuses on simplicity, readability and accessibility, the heavily templatized part of original PCL is not implemented in this library due to the limitation of cython. However, the templated cython headers are added and you can still write this part of code in C++ and then wrap the input and output in python easily with this library.  

------------------------

## Compatibility
- Written in Python 3 & Cython
- Tested on Python 3.5, Numpy>=1.10 (Temporarily)

------------------------

## Notes
The cython doesn't support a template technique which is similar to ["covariant"](https://en.wikipedia.org/wiki/Covariance_and_contravariance_(computer_science)) in its template support, so the code which need this technique is not wrapped or header-provided as stated above.

The library is under heavy construction, thus do not use it in productive codes. However, playing with it is totally welcome now, and it will be great to receive your issues, suggestions and pull requests!~