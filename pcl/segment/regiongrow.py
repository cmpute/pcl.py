'''
Implementation of following files:
    pcl/segmentation/include/pcl/segmentation/region_growing.h
    pcl/segmentation/include/pcl/segmentation/impl/region_growing.hpp
    pcl/segmentation/src/region_growing.cpp
'''

import math
from collections import deque
import numpy as np
from numpy.random import randint
from ..pointcloud import PointCloud
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
    def __init__(self, cloud=None, indices=None, normals=None):
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
        self._normals = None
        self._point_neighours = dict() # dict as 2d array
        self._point_labels = []
        self._normal_flag = True
        self._num_pts_in_segment = []
        self._clusters = []
        self._number_of_segments = 0

        self.input_normals = normals # input check

    @property
    def input_normals(self):
        '''
        Gets the normals.
        '''
        return self._normals

    @input_normals.setter
    def input_normals(self, value):
        '''
        Sets the normals.

        They are needed for the algorithm, so if no normals will be set,
        the algorithm would not be able to segment the points.
        '''
        if value is not None and not ('normal_x' in value.names and 'curvature' in value.names):
            raise ValueError('invalid input normals cloud')
        self._normals = value

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
        if self._normals is None or len(self._input) != len(self._normals):
            raise ValueError('invalid input normals cloud')

        if self._residual_flag:
            if self.residual_threshold <= 0:
                raise ValueError('residual threshold is not set')
        # no need to check curvature test

        if self.number_of_neighbours <= 0:
            raise ValueError('number of neighbours is not set')

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
        self._point_neighours = dict()
        for i_point, point_index in enumerate(self._indices):
            neighbours, _ = self.search_method.nearestk_search(i_point, self.number_of_neighbours)
            self._point_neighours[point_index] = neighbours

    def _apply_smooth_region_growing_algorithm(self):
        '''
        This function implements the algorithm described in the article
        *Segmentation of point clouds using smoothness constraint*
        by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
        '''
        num_of_pts = len(self._indices)
        self._point_labels = [-1] * len(self._input)
        point_residual = []

        if self._normal_flag:
            point_residual = np.argsort(self._normals['curvature'])
        else:
            point_residual = range(len(self._indices))
        seed_counter = 0
        seed = point_residual[seed_counter]

        segmented_pts_num = 0
        number_of_segments = 0
        while segmented_pts_num < num_of_pts:
            pts_in_segment = self._grow_region(seed, number_of_segments)
            segmented_pts_num += pts_in_segment
            self._num_pts_in_segment.append(pts_in_segment)
            number_of_segments += 1

            for i_seed in range(seed_counter + 1, num_of_pts):
                index = point_residual[i_seed]
                if self._point_labels[index] == -1:
                    seed = index
                    seed_counter = i_seed
                    break

    def _grow_region(self, initial_seed, segment_number):
        '''
        This method grows a segment for the given seed point. And returns the number of its points.

        # Parameters
        initial_seed : int
            Index of the point that will serve as the seed point
        segment_number : int
            Indicates which number this segment will have

        # Return
        count : int
            The number of points in the segment
        '''
        seeds = deque()
        seeds.append(initial_seed)
        self._point_labels[initial_seed] = segment_number

        num_pts_in_segment = 1

        while len(seeds) > 0:
            curr_seed = seeds.popleft()

            i_nghbr = 0
            while i_nghbr < self.number_of_neighbours and \
                  i_nghbr < len(self._point_neighours[curr_seed]):
                index = self._point_neighours[curr_seed][i_nghbr]
                if self._point_labels[index] != -1:
                    i_nghbr += 1
                    continue

                belong_to_segment, is_a_seed = self._validate_point(initial_seed, curr_seed, index)
                if not belong_to_segment:
                    i_nghbr += 1
                    continue

                self._point_labels[index] = segment_number
                num_pts_in_segment += 1

                if is_a_seed:
                    seeds.append(index)

                i_nghbr += 1

        return num_pts_in_segment

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
        is_a_seed = True
        cosine_threshold = math.cos(self.smoothness_threshold)
        initial_point = self._input[point].xyz
        initial_normal = self._normals[point].normal
        initial_seed_normal = self._normals[initial_seed].normal

        # check the angle between normals
        nghbr_normal = self._normals[nghbr].normal
        if self.smooth_mode_flag:
            if np.abs(np.dot(nghbr_normal, initial_normal)) > cosine_threshold:
                return False, is_a_seed
        else:
            if np.abs(np.dot(nghbr_normal, initial_seed_normal)) > cosine_threshold:
                return False, is_a_seed

        if self._curvature_flag and \
           self._normals[nghbr].to_ndarray('curvature') > self.curvature_threshold:
            is_a_seed = False

        nghbr_point = self._input[nghbr].xyz
        residual = np.abs(initial_normal.dot(initial_point - nghbr_point))
        if self._residual_flag and residual > self.residual_threshold:
            is_a_seed = False

        return True, is_a_seed

    def _assemble_regions(self):
        '''
        This function simply assembles the regions from list of point labels.
        Each cluster is an array of point indices.
        '''
        number_of_segments = len(self._num_pts_in_segment)
        number_of_points = len(self._input)
        self._clusters = [[]] * number_of_segments
        for i_point in range(number_of_points):
            segment_index = self._point_labels[i_point]
            if segment_index is not -1:
                self._clusters[segment_index].append(i_point)
        self._number_of_segments = number_of_segments

    def extract(self):
        '''
        This method launches the segmentation algorithm and returns the clusters that were
        obtained during the segmentation.

        # Returns
        clusters : list of indices (list of list of int)
            clusters that were obtained. Each cluster is an array of point indices.
        '''
        self._clusters = []
        self._point_neighours = dict()
        self._point_labels = []
        self._num_pts_in_segment = []
        self._number_of_segments = 0

        if not self._init_compute():
            self._deinit_compute()
            return None

        if not self._prepare_for_segmentation():
            self._deinit_compute()
            return None

        self._find_point_neighbours()
        self._apply_smooth_region_growing_algorithm()
        self._assemble_regions()

        cluster = []
        for cluster_iter in self._clusters:
            if self.min_cluster_size <= len(cluster_iter) and \
               len(cluster_iter) <= self.max_cluster_size:
                cluster.append(cluster_iter)

        # Converting to list of PointIndices, ignored here in python
        self._deinit_compute()
        return cluster

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
        if not self._init_compute():
            self._deinit_compute()
            return None

        if index in self._indices:
            if len(self._clusters) == 0:
                self._point_neighours = dict()
                self._point_labels = []
                self._num_pts_in_segment = []
                self._number_of_segments = 0

                if not self._prepare_for_segmentation:
                    self._deinit_compute()
                    return None

                self._find_point_neighbours()
                self._apply_smooth_region_growing_algorithm()
                self._assemble_regions()

            for cluster in self._clusters:
                if index in cluster:
                    return cluster

        return None

    def get_colored_cloud(self):
        '''
        If the cloud was successfully segmented, then function
        returns colored cloud (with only rgb fields). Otherwise it returns None.

        Points that belong to the same segment have the same color.
        But this function doesn't guarantee that different segments will have different color(it
        all depends on RNG). Points that were not listed in the indices array will have red color.
        '''
        if len(self._clusters) == 0:
            return None

        colors = randint(0, 256, size=(len(self._clusters), 3), dtype='u1')
        colordata = np.tile(np.array(255<<24, dtype='<u4'), len(self._clusters))
        colordata |= colors[:, 0] << 16 | colors[:, 1] << 8 | colors[:, 2]
        colordata.dtype = 'f4'

        colored_cloud = PointCloud(width=self._input.width, height=self._input.height,
                                   fields=[('rgb', 'f4')])
        self._input.copy_metadata(colored_cloud)

        for coloridx, cluster in enumerate(self._clusters):
            for index in cluster:
                colored_cloud[index] = colordata[coloridx]

        return colored_cloud

    def get_colored_cloud_rgba(self):
        '''
        If the cloud was successfully segmented, then function
        returns colored cloud (with only rgba fields). Otherwise it returns None.

        Points that belong to the same segment have the same color.
        But this function doesn't guarantee that different segments will have different color(it
        all depends on RNG). Points that were not listed in the indices array will have red color.
        '''
        if len(self._clusters) == 0:
            return None

        colors = randint(0, 256, size=(len(self._clusters), 3), dtype='u1')
        colordata = np.tile(np.array(255<<24, dtype='<u4'), len(self._clusters))
        colordata |= colors[:, 0] << 16 | colors[:, 1] << 8 | colors[:, 2]

        colored_cloud = PointCloud(width=self._input.width, height=self._input.height,
                                   fields=[('rgba', 'u4')])
        self._input.copy_metadata(colored_cloud)

        for coloridx, cluster in enumerate(self._clusters):
            for index in cluster:
                colored_cloud[index] = colordata[coloridx]

        return colored_cloud
