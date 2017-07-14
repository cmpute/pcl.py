'''
Implementation of following files:
    pcl/common/include/pcl/impl/pcl_base.hpp
    pcl/common/include/pcl/pcl_base.h
    pcl/common/include/pcl/common/centroid.h
    pcl/common/include/pcl/common/impl/centroid.hpp
    pcl/common/src/pcl_base.cpp
'''

import abc
import numpy as np
from .pointcloud import PointCloud

class _CloudBase(metaclass=abc.ABCMeta):
    '''
    Point Cloud Base class. Implements methods that are used by most PCL algorithms.
    '''
    def __init__(self, cloud=None, indices=None):
        self._input = cloud
        if not indices and cloud is not None:
            self._indices = range(len(cloud))
            self.__fake_indices = True
        else:
            self._indices = indices
            self.__fake_indices = False
            if cloud is not None and indices is not None:
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
        self.__check_indices()

    @property
    def indices(self):
        '''
        Get a reference to the vector of indices used.
        '''
        assert self._indices is not None or self._input is None
        return self._indices

    @indices.setter
    def indices(self, value):
        '''
        Provide a reference to the vector of indices that represents the input data.
        '''
        if value is not None:
            self._indices = list(value)
        else:
            self._indices = None
        self.__check_indices()

    def __check_indices(self):
        if self._input is None:
            # clear indice
            if self._indices is not None and len(self._indices) > 0:
                self._indices = None
            self.__fake_indices = False
            return

        if self._indices is None:
            self._indices = range(len(self._input))
            self.__fake_indices = True
        elif self.__fake_indices and len(self._indices) != self.input_cloud:
            self._indices = range(len(self._input))

    def _init_compute(self):
        '''
        Initialize computation. Must be called before processing starts.
        '''
        if self._input is None:
            raise ValueError('null input point cloud')
        return True

    def _deinit_compute(self):
        '''
        UnInitialize computation. Must be called after processing ends
        '''
        pass

######################
##### Centroids ######
######################

def compute_covariance_matrix(cloud, centroid):
    '''
    Compute the 3x3 covariance matrix of a given set of points.

    The covariance matrix is not normalized with the number of
    points. For a normalized covariance, please use
    compute_normalized_covariance_matrix.

    # Parameters
    cloud : PointCloud
        The input point cloud
    centroid : Point
        The centroid of the set of points in the cloud

    # Returns
    covariance_matrix : 3x3 matrix
        The resultant 3x3 covariance matrix
    '''
    pass

def compute_mean_and_covariance_matrix(cloud, indices=None, bias=True):
    '''
    Compute the normalized 3x3 covariance matrix and the centroid of a given set of points
    in a single loop.

    # Parameters
    cloud : PointCloud
        The input point cloud
    indices : list of int
        Subset of points given by their indices
    bias : bool
        If False, the covariance is un-biased (normalized by n-1)
        If True, the covariance is biased (normalized by n)

    # Returns
    covariance_matrix : 3x3 matrix
        The resultant 3x3 covariance matrix
    centroid : Point
        The centroid of the set of points in the cloud
    '''
    if indices is not None:
        cloud = cloud[indices]
    # Filter invalid points
    # cloud = cloud[~np.isnan(np.sum(cloud.xyz, axis=1))].data
    cloud = cloud.data

    # a bit faster than np.cov if compute centroid and covariance at the same time
    # testing shows this method is faster at ratio about 30%, however precision has lost a bit
    accu = dict()
    cloudx = cloud['x']
    cloudy = cloud['y']
    cloudz = cloud['z']
    coef_xx = np.mean(cloudx * cloudx)
    coef_xy = np.mean(cloudx * cloudy)
    coef_xz = np.mean(cloudx * cloudz)
    coef_yy = np.mean(cloudy * cloudy)
    coef_yz = np.mean(cloudy * cloudz)
    coef_zz = np.mean(cloudz * cloudz)
    coef_x = np.mean(cloudx)
    coef_y = np.mean(cloudy)
    coef_z = np.mean(cloudz)
    centroid = [coef_x, coef_y, coef_z, 1] # homogeneous form

    if not bias:
        for key in accu:
            accu[key] *= len(cloud)/(len(cloud) - 1)
    cov = np.zeros((3, 3))
    cov[0, 0] = coef_xx - coef_x**2
    cov[1, 1] = coef_yy - coef_y**2
    cov[2, 2] = coef_zz - coef_z**2
    cov[0, 1] = cov[1, 0] = coef_xy - coef_x*coef_y
    cov[0, 2] = cov[2, 0] = coef_xz - coef_x*coef_z
    cov[1, 2] = cov[2, 1] = coef_yz - coef_y*coef_z

    return cov, centroid
