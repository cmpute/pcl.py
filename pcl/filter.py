'''
Implementation of following files:
    pcl/filters/include/pcl/filters/impl/filter.hpp
    pcl/filters/include/pcl/filters/impl/filterindices.hpp
    pcl/filters/include/pcl/filters/filter.h
    pcl/filters/include/pcl/filters/filterindices.h
    pcl/filters/include/pcl/filters/clipper3D.h
    pcl/filters/src/filter.cpp
    pcl/filters/src/filterindices.cpp

Following files are abandoned:
'''

import abc
import numpy as np
from .common import _CloudBase
from .pointcloud import PointCloud

########### General Filters ###########
class Filter(_CloudBase, metaclass=abc.ABCMeta):
    '''
    Filter represents the base filter class. All filters must inherit from this interface.
    '''
    def __init__(self, extract_removed=True, cloud=None, indices=None):
        super().__init__(cloud, indices)
        self._remove_indices = []
        self._extract_removed_indices = extract_removed

    @property
    def removed_indices(self):
        '''
        Get the point indices being removed
        '''
        return self._remove_indices

    def filter(self):
        '''
        Calls the filtering method and returns the filtered dataset in output.

        # Returns
        output : PointCloud
            The resultant filtered point cloud dataset
        '''
        return self._apply_filter()

    @abc.abstractmethod
    def _apply_filter(self):
        '''
        Abstract filter method.

        The implementation needs to set output.{points, width, height, is_dense}.
        '''
        pass

class BilateralFilter(Filter):
    '''
    A bilateral filter implementation for point cloud data. Uses the intensity data channel.

    For more information please see

    C. Tomasi and R. Manduchi. *Bilateral Filtering for Gray and Color Images.*
    In Proceedings of the IEEE International Conference on Computer Vision,
    1998.
    '''
    pass # TODO: Not Implemented

####### Organized Cloud Filters #######
class MedianFilter(Filter):
    '''
    Implementation of the median filter.

    The median filter is one of the simplest and wide-spread image processing filters. It is known
    to perform well with "shot"/impulse noise (some individual pixels having extreme values), it
    does not reduce contrast across steps in the function (as compared to filters based on
    averaging), and it is robust to outliers. Furthermore, it is simple to implement and efficient,
    as it requires a single pass over the image. It consists of a moving window of fixed size that
    replaces the pixel in the center with the median inside the window.

    This algorithm filters only the depth (z-component) of **organized** and untransformed
    (i.e., in camera coordinates) point clouds. An error will be outputted if an unorganized cloud
    is given to the class instance.
    '''
    pass # TODO: Not Implemented

class FastBilateralFilter(Filter):
    '''
    Implementation of a fast bilateral filter for smoothing depth information in organized
    point clouds

    Based on the following paper:

    Sylvain Paris and Frédo Durand "A Fast Approximation of the Bilateral Filter using a Signal
    Processing Approach" European Conference on Computer Vision (ECCV'06)
    '''
    pass # TODO: Not Implemented

########### Binary Filters ############
class FilterIndices(Filter, metaclass=abc.ABCMeta):
    '''
    FilterIndices represents the base class for filters that are about binary point removal.

    All derived classes have to implement the filter() methods. Ideally they also make use of
    the negative, keep_organized and _extract_removed_indices systems. The distinguishment
    between the negative and _extract_removed_indices systems only makes sense if the class
    automatically filters non-finite entries in the filtering methods (recommended).
    '''
    def __init__(self, extract_removed=True, cloud=None, indices=None):
        super().__init__(extract_removed, cloud, indices)
        self.negative = False
        self.keep_organized = False
        self.user_filter_value = float('nan')

    def filter_indices(self, cloud):
        '''
        Calls the filtering method and returns the filtered point cloud indices.

        # Returns
        indices : list of int
            The resultant filtered point cloud indices
        '''
        return self._apply_filter_indices(cloud)

    @abc.abstractmethod
    def _apply_filter_indices(self, cloud):
        pass

class ExtractIndices(FilterIndices):
    '''
    ExtractIndices extracts a set of indices from a point cloud.
    '''
    pass # TODO: Not Implemented

class ModelOutlierRemoval(FilterIndices):
    '''
    ModelOutlierRemoval filters points in a cloud based on the distance between model and point.

    The filter iterates through the entire input once, automatically filtering non-finite points
    and the points outside the model specified by sample_consensus_model and the threshold
    specified by threhold_function.
    '''
    pass # TODO: Not Implemented

class PassThrough(FilterIndices):
    '''
    PassThrough passes points in a cloud based on constraints for one particular field of the
    point type.

    The filter iterates through the entire input once, automatically filtering non-finite points
    and the points outside the interval specified by filter_limits, which applies only to the field
    specified by filter_field_name.
    '''
    pass # TODO: Not Implemented

class LocalMaximum(FilterIndices):
    '''
    LocalMaximum downsamples the cloud, by eliminating points that are locally maximal.

    The LocalMaximum class analyzes each point and removes those that are found to be locally
    maximal with respect to their neighbors (found via radius search). The comparison is made
    in the z dimension only at this time.
    '''
    pass # TODO: Not Implemented

############# Voxel Grid ##############
class VoxelGrid(Filter):
    '''
    VoxelGrid assembles a local 3D grid over a given PointCloud,
    and downsamples + filters the data.

    The VoxelGrid class creates a **3D voxel grid** (think about a voxel
    grid as a set of tiny 3D boxes in space) over the input point cloud data.
    Then, in each *voxel* (i.e., 3D box), all the points present will be
    approximated (i.e., *downsampled*) with their centroid. This approach is
    a bit slower than approximating them with the center of the voxel, but it
    represents the underlying surface more accurately.
    '''
    def __init__(self, extract_removed=True, cloud=None, indices=None):
        super().__init__(extract_removed, cloud, indices)
        self._leaf_size = np.zeros(4)
        self._inverse_leaf_size = np.zeros(4)
        self.downsample_all_data = True
        self.save_leaf_layout = False
        self._leaf_layout = None
        self.min_box_coordinates = np.zeros(4)
        self.max_box_coordinates = np.zeros(4)
        self._div_b = np.zeros(4)
        self._divb_mul = np.zeros(4)
        self.filter_field_name = ''
        self._filter_limit_min = float('-inf')
        self._filter_limit_max = float('inf')
        self.filter_limit_negative = False
        self.min_points_per_voxel = 0

    @property
    def leaf_size(self):
        '''
        Get the voxel grid leaf size.
        '''
        return self._leaf_size

    @leaf_size.setter
    def leaf_size(self, value):
        '''
        Set the voxel grid leaf size.

        # Parameters
        value : array [x, y, z, 1]
        '''
        self._leaf_size = value[:4]
        if self._leaf_size[3] == 0:
            self._leaf_size[3] = 1
        self._inverse_leaf_size = np.ones(4)/self._leaf_size

    @property
    def num_divisions(self):
        '''
        Get the number of divisions along all 3 axes (after filtering is performed).
        '''
        return self._div_b[:2]

    @property
    def division_multiplier(self):
        '''
        Get the multipliers to be applied to the grid coordinates in
        order to find the centroid index (after filtering is performed).
        '''
        return self._divb_mul[:3]

    @property
    def filter_limits(self):
        '''
        Get the field filter limits (min, max) set by the user. The default values are -inf, inf.
        '''
        return self._filter_limit_min, self._filter_limit_max

    @filter_limits.setter
    def filter_limits(self, value):
        '''
        Set the field filter limits. All points having field values outside this interval will
        be discarded.
        '''
        self._filter_limit_min, self._filter_limit_max = value

    # TODO: Not Implemeted completely yet

############## Clippers ###############
class Clipper(metaclass=abc.ABCMeta):
    '''
    Base class for 3D clipper objects
    '''
    @abc.abstractmethod
    def clip_point(self, point):
        '''
        Interface to clip a single point

        # Parameters
        point : point
            The point to check against

        # Returns
        result : bool
            True if point still exists, False if it's clipped
        '''
        pass

    @abc.abstractmethod
    def clip_line_segment(self, point1, point2):
        '''
        Interface to clip a line segment given by two end points.
        The order of the end points is unimportant and will sty the same after clipping.

        # Parameters
        point1 : point
            Start point of the line segment
        point2 : point
            End point of the line segment

        # Returns
        result : bool
            True if the clipped line is not empty, thus the parameters are still valid,
            False if line completely outside clipping space
        '''
        pass

    @abc.abstractmethod
    def clip_planar_polygon(self, polygon):
        '''
        Interface to clip a planar polygon given by an ordered list of points

        # Parameters
        polygon : list of points or point cloud
            The polygon in any direction (ccw or cw) but ordered, thus two neighboring points
            define an edge of the polygon

        # Returns
        clipped_polygon : list of points or point cloud
            The clipped polygon
        '''
        pass

    @abc.abstractmethod
    def clip_point_cloud(self, cloud, indices=None):
        '''
        Interface to clip a point cloud

        # Parameters
        cloud : PointCloud
            Input point cloud
        indices : list of int
            The indices of points in the point cloud to be clipped.

        # Returns
        clipped : list of int
            Indices of points that remain after clipping the input cloud
        '''
        pass

    @abc.abstractmethod
    def clone(self):
        '''
        Polymorphic method to clone the underlying clipper with its parameters.

        # Returns
        The new clipper object from the specific subclass with all its parameters.
        '''
        pass

class BoxClipper(Clipper):
    '''
    Implementation of a box clipper in 3D. Actually it allows affine transformations,
    thus any parallelepiped in general pose.

    The affine transformation is used to transform the point before clipping it using the unit
    cube centered at origin and with an extend of -1 to +1 in each dimension
    '''
    pass # TODO: Not Implemented

class PlaneClipper(Clipper):
    '''
    Implementation of a plane clipper in 3D
    '''
    pass # TODO: Not Implementd

Filter.register(BilateralFilter)
Filter.register(MedianFilter)
Filter.register(FastBilateralFilter)
Filter.register(FilterIndices)
FilterIndices.register(ExtractIndices)
FilterIndices.register(ModelOutlierRemoval)
FilterIndices.register(PassThrough)
FilterIndices.register(LocalMaximum)
Filter.register(VoxelGrid)
Clipper.register(BoxClipper)
Clipper.register(PlaneClipper)
