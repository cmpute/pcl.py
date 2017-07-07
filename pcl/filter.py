'''
Implementation of following files:
    pcl/filters/include/pcl/filters/impl/filter.hpp
    pcl/filters/include/pcl/filters/filter.h
    pcl/filters/src/filter.cpp

Following files are abandoned:
'''

import abc
import numpy as np
from .common import _CloudBase
from .pointcloud import PointCloud

class Filter(_CloudBase, metaclass=abc.ABCMeta):
    '''
    Filter represents the base filter class. All filters must inherit from this interface.
    '''
    pass
