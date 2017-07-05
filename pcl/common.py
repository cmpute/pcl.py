'''
Implementation of following files:
    pcl/common/include/pcl/impl/pcl_base.hpp
    pcl/common/include/pcl/pcl_base.h
    pcl/common/src/pcl_base.cpp
'''

import abc
from .pointcloud import PointCloud

class _CloudBase(metaclass=abc.ABCMeta):
    '''
    Point Cloud Base class. Implements methods that are used by most PCL algorithms.
    '''
    def __init__(self, cloud=None, indices=None):
        self._input = cloud
        self._indices = indices
        if not cloud and not indices:
            if len(indices) > len(cloud):
                raise ValueError('Invalid index vector (too long)')

    @property
    def input_cloud(self):
        '''
        Get a reference to the input point cloud dataset.
        '''
        return self._input

    @input_cloud.setter
    def input_cloud(self, value):
        '''
        Provide a reference to the input dataset
        '''
        self._input = PointCloud(value, copy=False)

    @property
    def indices(self):
        '''
        Get a reference to the vector of indices used.
        '''
        return self._indices

    @indices.setter
    def indices(self, value):
        '''
        Provide a reference to the vector of indices that represents the input data.
        '''
        self._indices = list(value)
