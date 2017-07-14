'''
Implementation of following files:
    pcl/features/include/pcl/features/normal_3d.h
    pcl/features/include/pcl/features/impl/normal_3d.hpp
    pcl/features/src/normal_3d.cpp
'''

import numpy as np
from ..pointcloud import PointCloud
from ..common import compute_mean_and_covariance_matrix
from .feature import Feature

def compute_point_normal(cloud, indices):
    '''
    Compute the Least-Squares plane fit for a given set of points, using their indices,
    and return the estimated plane parameters together with the surface curvature.

    # Parameters
    cloud : PointCloud
        The input Point Cloud
    indices : list of int
        The point cloud indices that need to be used

    # Returns
    plane_parameters : list of float
        The plane parameters as: a, b, c, d (ax + by + cz + d = 0)
    curvature : float
        the estimated surface curvature as a measure of λ_0 / (λ_0 + λ_1 + λ_2)
    '''
    if len(indices) < 3:
        raise ValueError('not enough points for computing normal')

    # Computing normal vector and curvature using PCA
    covariance_matrix, xyz_centroid = compute_mean_and_covariance_matrix(cloud, indices)
    eigen_value, eigen_vector = np.linalg.eigh(covariance_matrix)
    smallest = np.argmin(eigen_value)

    plane_parameters = eigen_vector[:, smallest].tolist()
    plane_parameters.append(-np.dot(plane_parameters + [0], xyz_centroid))
    eigen_sum = np.sum(eigen_value)
    if eigen_sum != 0:
        curvature = np.abs(eigen_value[smallest] / eigen_sum)
    else:
        curvature = 0

    return plane_parameters, curvature

class NormalEstimation(Feature):
    '''
    NormalEstimation estimates local surface properties (surface normals and curvatures)at each
    3D point.
    '''
    def __init__(self, cloud=None, indices=None):
        super().__init__(cloud, indices)
        self._view_point = np.array([0, 0, 0])
        self._covariance_matrix = None
        self._xyz_centroid = None
        self._use_sensor_origin = True

    @property
    def input_cloud(self):
        return super().input_cloud

    @input_cloud.setter
    def input_cloud(self, value):
        super().input_cloud = value
        if self._use_sensor_origin:
            self._view_point = self._input.sensor_origin

    @property
    def view_point(self):
        '''
        Get the viewpoint (x, y, z)

        If the viewpoint is set manually using the setViewPoint method, this method will return
        the set view point coordinates. If an input cloud is set, it will return the sensor origin
        otherwise it will return the origin (0, 0, 0)
        '''
        return self._view_point

    @view_point.setter
    def view_point(self, value):
        '''
        Set the viewpoint.
        '''
        self._view_point = np.array(value)

    def use_sensor_origin_as_view_point(self):
        '''
        Sets the sensor origin as a user given viewpoint should be used.
        After this method, the normal estimation method uses the sensor origin of the input cloud.
        to use a user defined view point, use the property view_point
        '''
        self._use_sensor_origin = True
        if self._input is not None:
            self._view_point = self._input.sensor_origin
        else:
            self._view_point = np.array([0, 0, 0])

    def _compute_feature(self):
        dtype = [('normal_x', 'f8'), ('normal_y', 'f8'), ('normal_z', 'f8'), ('curvature', 'f8')]
        params = np.empty((len(self._indices),), dtype=dtype)
        for count, idx in enumerate(self._indices):
            nn_indices, _ = self._search_for_neighbours(idx, self._search_parameter)
            plane_param, curvature = compute_point_normal(self._surface, nn_indices)

            # flip_normal_towards_viewpoint
            if np.dot(self._view_point - self._input[idx].xyz, plane_param[:3]) < 0:
                params[count] = -plane_param[0], -plane_param[1], -plane_param[2],\
                                curvature
            else:
                params[count] = plane_param[0], plane_param[1], plane_param[2],\
                                curvature

        output = PointCloud(params, fields=dtype)
        output.copy_metadata(self._input)
        return output
