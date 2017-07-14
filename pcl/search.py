'''
Implementation of following files:
    pcl/search/include/pcl/search/search.h
    pcl/search/include/pcl/search/brute_force.h
    pcl/search/include/pcl/search/impl/search.hpp
    pcl/search/include/pcl/search/impl/brute_force.hpp
    pcl/search/src/search.cpp
    pcl/search/src/brute_force.cpp
'''

import abc
import numpy as np
from .common import _CloudBase

class Search(_CloudBase, metaclass=abc.ABCMeta):
    '''
    Generic search class. All search wrappers must inherit from this.

    Each search method must implement 2 different types of search:
      - nearestKSearch - search for K-nearest neighbors.
      - radiusSearch - search for all nearest neighbors in a sphere of a given radius

    The input to each search method can be given in 2 different ways:
      - as a query point
      - as an index

    For the latter option, it is assumed that the user specified the input
    via a setInputCloud () method first.

    In case of an error, all methods are supposed to return 0 as the number of neighbors found.

    libpcl_search deals with three-dimensional search problems. For higher
    level dimensional search, please refer to the libpcl_kdtree module.
    '''

    def __init__(self, cloud=None, indices=None, sort_results=False):
        super().__init__(cloud, indices)
        self._sort_results = sort_results

    @property
    def sort_results(self):
        '''
        Gets whether the results should be sorted (ascending in the distance) or not

        If false, the results may be returned in any order.
        '''
        return self._sort_results

    @sort_results.setter
    def sort_results(self, value):
        '''
        Sets whether the results should be sorted (ascending in the distance) or not

        If false, the results may be returned in any order.
        '''
        self._sort_results = value

    @abc.abstractmethod
    def nearestk_search(self, point, k):
        '''
        Search for the k-nearest neighbors for the given query point.

        # Parameters
        point : point or int
            The given query point. If it is a integer, then query point is the one in the cloud
            with the parameter as index
        k : int
            The number of neighbors to search for

        # Returns
        k_indices : list of int
            The resultant indices of the neighboring points
        k_sqr_distances : list of float
            The resultant squared distances to the neighboring points
        '''
        pass

    def radius_search(self, point, radius, max_nn=0):
        '''
        Search for the k-nearest neighbors for the given query point.

        # Parameters
        point : point or int
            The given query point. If it is a integer, then query point is the one in the cloud
            with the parameter as index
        radius : float
            The radius of the sphere bounding all of p_q's neighbors
        max_nn : int
            if given, bounds the maximum returned neighbors to this value. If max_nn is set to
            0 or to a number higher than the number of points in the input cloud, all neighbors
            in radius will be returned.

        # Returns
        k_indices : list of int
            The resultant indices of the neighboring points
        k_sqr_distances : list of float
            The resultant squared distances to the neighboring points
        '''
        pass

class BruteForceSearch(Search):
    '''
    Implementation of a simple brute force search algorithm.
    '''
    def __init__(self, cloud=None, indices=None, sort_results=False):
        super().__init__(cloud, indices, sort_results)

    def nearestk_search(self, point, k):
        '''
        Search for the k-nearest neighbors for the given query point.

        # Parameters
        point : point or int
            The given query point. If it is a integer, then query point is the one in the input
            cloud with the parameter as index
        k : int
            The number of neighbors to search for

        # Returns
        k_indices : list of int
            The resultant indices of the neighboring points
        k_sqr_distances : list of float
            The resultant squared distances to the neighboring points
        '''
        if k < 1:
            return [], []
        if isinstance(point, int):
            point = self._input[point].xyz

        # nan values won't break the method
        indices = np.array(self._indices, copy=False)
        points = self._input.xyz[indices]

        dist = points - point
        dist = np.sum(dist * dist, axis=1)
        if len(dist) <= k:
            k_indices = indices
            k_distances = dist
        else:
            parts = dist.argpartition(k)[:k]
            k_indices = indices[parts]
            k_distances = dist[parts]

        if self._sort_results:
            seq = k_distances.argsort()
            k_indices = k_indices[seq]
            k_distances = k_distances[seq]

        return k_indices, np.sqrt(k_distances)

    def radius_search(self, point, radius, max_nn=0):
        '''
        Search for the k-nearest neighbors for the given query point.

        # Parameters
        point : point or int
            The given query point. If it is a integer, then query point is the one in the input
            cloud with the parameter as index
        radius : float
            The radius of the sphere bounding all of p_q's neighbors
        max_nn : int
            if given, bounds the maximum returned neighbors to this value. If max_nn is set to
            0 or to a number higher than the number of points in the input cloud, all neighbors
            in radius will be returned.

        # Returns
        k_indices : list of int
            The resultant indices of the neighboring points
        k_sqr_distances : list of float
            The resultant squared distances to the neighboring points
        '''
        if isinstance(point, int):
            point = self._input[point].xyz

        # nan values won't break the method
        indices = np.array(self._indices, copy=False)
        points = self._input.xyz[indices]

        dist = points - point
        dist = np.sum(dist * dist, axis=1)
        predicate = dist < radius * radius
        k_indices = indices[predicate]
        k_distances = dist[predicate]

        if self._sort_results:
            seq = k_distances.argsort()
            k_indices = k_indices[seq]
            k_distances = k_distances[seq]

        return k_indices, np.sqrt(k_distances)

DefaultSearch = BruteForceSearch
DefaultOrganizedSearch = BruteForceSearch
