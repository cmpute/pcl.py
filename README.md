# pcl.py [![Build Status](https://api.travis-ci.com/cmpute/pcl.py.svg?branch=master)](https://travis-ci.com/cmpute/pcl.py)
<!-- [![Coverage Status](https://coveralls.io/repos/github/cmpute/pcl.py/badge.svg)](https://coveralls.io/github/cmpute/pcl.py) -->

Cython bindings of Point Cloud Library (PCL)

## Principles
Implement a easy-using cython library to process point cloud, combined with scipy and numpy. This library wraps `PCLPointCloud2` class into python and users can pass data from numpy to `PointCloud<PointT>` easily with this library and headers.

Nevertheless, this library focuses on simplicity, readability and accessibility, the heavily templatized part of original PCL is not implemented in this library due to the limitation of cython. However, the templated cython headers are added and you can still write this part of code in C++ and then wrap the input and output in python easily with this library.  

### Interface
The major classes in this library are `pcl.PointCloud`, `pcl.Visualizer`. Most methods from C++ library are directly wrapped into Python, while methods with `get` and `set` prefix are changed into Python object properties.

### Advantage over [python-pcl](https://github.com/strawlab/python-pcl/)
- PointCloud is stored in PCLPointCloud2 instead of PointCloud<PointT>. Thus the cloud can store various kind of point type
- Cython files are in the same structure as C++ library, which means the code you write in cython can be easily transferred to C++
- ROS support
- Templated algorithms are implemented with most used point type, the other fields will be reserved.
- Using CMake to build extension, thus this library can be compiled upon different settings of PCL

------------------------

## Compatibility
- Written in Python 3 & Cython
- Tested on Python 3.5, Numpy>=1.10 (Temporarily)
- Tested on PCL 1.8.0, adding more compatibility is pending.

------------------------

## Installation

- Installation from PyPI: `pip install pcl-py`
- Installation from source: `python setup.py install`
- Installation from source in-place: `python setup.py develop`

------------------------

## Notes
The cython doesn't support a template technique similar to ["covariant"](https://en.wikipedia.org/wiki/Covariance_and_contravariance_(computer_science)) in its template support, so the code which need this technique is not wrapped or header-provided as stated above.

The library is under heavy construction, thus do not use it in productive codes. However, playing with it is totally welcome now, and it will be great to receive your issues, suggestions and pull requests!~

-------------------------

# Usage

```python
import pcl
import numpy as np

cloud = pcl.PointCloud([(1,2,3), (4,5,6)])
centroid = np.mean(cloud.xyz, axis=0)
```

Please check test codes for more usage examples

## Common failure
`pcl.PointCloud(...).xyz`: This will return uninitialized memory. This is a known issue but I don't know how to fix it...
`pcl.PointCloud([[1,2,3], [4,5,6]])`: Only list of tuples are accepted for initialization
