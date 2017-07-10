'''
Implementation of following files:
    pcl/segmentation/include/pcl/segmentation/region_growing.h
    pcl/segmentation/include/pcl/segmentation/impl/region_growing.hpp
    pcl/segmentation/src/region_growing.cpp
'''

import math
import numpy as np
from ..common import _CloudBase
from ..search import DefaultSearch

class RegionGrowing(_CloudBase):
    '''
    Implements the well known Region Growing algorithm used for segmentation.

    Description can be found in the article
    *Segmentation of point clouds using smoothness constraint*
    by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.

    In addition to residual test, the possibility to test curvature is added.
    '''
    def __init__(self, cloud=None, indices=None):
        super().__init__(cloud, indices)
        self.min_cluster_size = 1
        self.max_cluster_size = float('inf')
        self.smooth_mode_flag = True
        self._curvature_flag = True
        self._residual_flag = False
        self.smoothness_threshold = 30 / 180 * math.pi # theta_threshold_
        self.residual_threshold = 0.05
        self.curvature_threshold = 0.05
        self.number_of_neighbours = 30 # neighbour_number_
        self.search_method = None
        self.input_normals = None
        self._point_neighours = []
        self._point_labels = []
        self._normal_flag = True
        self._num_pts_in_segment = []
        self._clusters = []
        self._number_of_segments = 0

    @property
    def residual_test_flag(self):
        '''
        Returns the flag that signalize if the residual test is turned on/off.
        '''
        return self._residual_flag

    @residual_test_flag.setter
    def residual_test_flag(self, value):
        '''
        Allows to turn on/off the residual test.

        Note that at least one test
        (residual or curvature) must be turned on. If you are turning residual test off
        then curvature test will be turned on automatically.
        '''
        self._residual_flag = value
        if not self._curvature_flag and not self._residual_flag:
            self._curvature_flag = True

    @property
    def curvature_test_flag(self):
        '''
        Returns the flag that signalize if the curvature test is turned on/off.
        '''
        return self._curvature_flag

    @curvature_test_flag.setter
    def curvature_test_flag(self, value):
        '''
        Allows to turn on/off the curvature test.

        Note that at least one test
        (residual or curvature) must be turned on. If you are turning curvature test off
        then residual test will be turned on automatically.
        '''
        self._curvature_flag = value
        if not self._curvature_flag and not self._residual_flag:
            self._residual_flag = True

    def _prepare_for_segmentation(self):
        '''
        This method simply checks if it is possible to execute the segmentation algorithm with
        the current settings. If it is possible then it returns true.
        '''
        if self._input is None:
            return False

        if self.input_normals is None or len(self._input) != len(self.input_normals):
            return False

        if self._residual_flag:
            if self.residual_threshold <= 0:
                return False
        # no need to check curvature test

        if self.number_of_neighbours <= 0:
            return False

        if self.search_method is None:
            self.search_method = DefaultSearch(self._input, self._indices)
        else:
            self.search_method.input_cloud = self._input

        return True

    def _find_point_neighbours(self):
        '''
        This method finds KNN for each point and saves them to the array
        because the algorithm needs to find KNN a few times.
        '''
        for i_point, point_index in enumerate(self._indices):
            neighbours, _ = self.search_method.nearestk_search(i_point, self.number_of_neighbours)
            self._point_neighours[point_index] = neighbours

    def _apply_smooth_region_growing_algorithm(self):
        '''
        This function implements the algorithm described in the article
        *Segmentation of point clouds using smoothness constraint*
        by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
        '''
        pass # TODO: Not Implemented

    def _grow_region(self, initial_seed, segment_number):
        '''
        This method grows a segment for the given seed point. And returns the number of its points.

        # Parameters
        initial_seed : int
            Index of the point that will serve as the seed point
        segment_number : int
            Indicates which number this segment will have
        '''
        pass # TODO: Not Implemented

    def _validate_point(self, initial_seed, point, nghbr):
        '''
        This function is checking if the point with index 'nghbr' belongs to the segment.
        If so, then it returns true. It also checks if this point can serve as the seed.

        # Parameters
        initial_seed : int
            Index of the initial point that was passed to the _grow_region() function
        point : int
            Index of the current seed point
        nghbr : int
            Index of the point that is neighbour of the current seed

        # Returns
        valid : bool
            Whether the point with index 'nghbr' belongs to the segment.
        is_a_seed : bool
            This value is set to true if the point with index 'nghbr' can serve as the seed
        '''
        pass # TODO: Not Implemented

    def _assemble_regions(self):
        '''
        This function simply assembles the regions from list of point labels.
        Each cluster is an array of point indices.
        '''
        pass # TODO: Not Implemented

    def extract(self):
        '''
        This method launches the segmentation algorithm and returns the clusters that were
        obtained during the segmentation.

        # Returns
        clusters : list of indices (list of list of int)
            clusters that were obtained. Each cluster is an array of point indices.
        '''
        pass # TODO: Not Implemented

    def get_segment_from_point(self, index):
        '''
        For a given point this function builds a segment to which it belongs and returns this
        segment.

        # Parameters
        index : int
            Index of the initial point which will be the seed for growing a segment.

        # Returns
        cluster : indices (list of int)
            cluster to which the point belongs.
        '''
        pass # TODO: Not Implemented

    def get_colored_cloud(self):
        '''
        If the cloud was successfully segmented, then function
        returns colored cloud. Otherwise it returns an empty pointer.
        Points that belong to the same segment have the same color.
        But this function doesn't guarantee that different segments will have different color(it
        all depends on RNG). Points that were not listed in the indices array will have red color.
        '''
        pass # TODO: Not Implemented

    def get_colored_cloud_rgba(self):
        '''
        If the cloud was successfully segmented, then function
        returns colored cloud. Otherwise it returns an empty pointer.
        Points that belong to the same segment have the same color.
        But this function doesn't guarantee that different segments will have different color(it
        all depends on RNG). Points that were not listed in the indices array will have red color.
        '''
        pass # TODO: Not Implemented
