'''
Implementation of following files:
    pcl/features/include/pcl/features/feature.h
    pcl/features/include/pcl/features/impl/feature.hpp
    pcl/features/src/normal_3d.cpp
'''

import abc
from ..pointcloud import PointCloud
from ..common import _CloudBase
from ..search import DefaultSearch, DefaultOrganizedSearch

class Feature(_CloudBase, metaclass=abc.ABCMeta):
    '''
    Feature represents the base feature class. Some generic 3D operations that
    are applicable to all features are defined here as static methods.

    The convention for a feature descriptor is:
    - if the nearest neighbors for the query point at which the descriptor is to be computed
      cannot be determined, the descriptor values will be set to NaN (not a number)
    - it is impossible to estimate a feature descriptor for a point that doesn't have finite
      3D coordinates. Therefore, any point that has NaN data on x, y, or z, will most likely
      have its descriptor set to NaN.
    '''
    def __init__(self, cloud=None, indices=None):
        super().__init__(cloud, indices)
        self._search_method_surface = None
        self._surface = None
        self.search_method = None
        self._search_parameter = 0
        self.search_radius = 0
        self.search_k = 0
        self._fake_surface = False

    @property
    def search_surface(self):
        '''
        Get a pointer to the surface point cloud dataset.
        '''
        return self._surface

    @search_surface.setter
    def search_surface(self, cloud):
        '''
        Provide a pointer to a dataset to add additional information
        to estimate the features for every point in the input dataset.
        This is optional, if this is not set, it will only use the data in the
        input cloud to estimate the features.
        This is useful when you only need to compute the features for a downsampled cloud.
        '''
        self._surface = cloud
        self._fake_surface = False

    @property
    def search_parameter(self):
        '''
        Get the internal search parameter.
        '''
        return self._search_parameter

    def compute(self):
        '''
        Base method for feature estimation for all points given in
        <input_cloud, indices> using the surface in search_surface
        and the spatial locator in search_method

        # Returns
        output : PointCloud
            The resultant point cloud model dataset containing the estimated features
            Metadata should be already copied
        '''
        if not self._init_compute():
            return None

        output = self._compute_feature()
        if len(self._indices) == len(self._input):
            output.width = self._input.width
            # output.height = self._input.height

        self._deinit_compute()
        return output

    def _search_for_neighbours(self, index, parameter):
        '''
        Search for k-nearest neighbors using the spatial locator from
        search_method, and the given surface from search_surface.
        '''
        # XXX There's bug in PCL, PCL uses input cloud to search, however according to usage of
        # search surface, the input surface cloud should be the one to be searched
        return self._search_method_surface(index, parameter)

    @abc.abstractmethod
    def _compute_feature(self, cloud):
        '''
        Abstract feature estimation method.

        # Parameter
        cloud : PointCloud
            Initial null point cloud, the resultant features should fill it after.
        '''
        pass

    def _init_compute(self):
        if not super()._init_compute():
            return False

        if len(self._input) == 0:
            self._deinit_compute()
            return False

        # If no search surface has been defined, use the input dataset as the search surface itself
        if self._surface is None:
            self._fake_surface = True
            self._surface = self._input

        if self.search_method is None:
            if self._surface.is_organized and self._input.is_organized:
                self.search_method = DefaultOrganizedSearch()
            else:
                self.search_method = DefaultSearch()
        self.search_method.input_cloud = self._surface

        if self.search_radius != 0:
            if self.search_k != 0:
                raise ValueError('both radius (%f) and K (%d) defined!' %
                                 (self.search_radius, self.search_k))
            else:
                self._search_parameter = self.search_radius
                self._search_method_surface = self.search_method.radius_search
        else:
            if self.search_k != 0:
                self._search_parameter = self.search_k
                self._search_method_surface = self.search_method.nearestk_search
            else:
                raise ValueError('neither radius nor K defined!')

        return True

    def _deinit_compute(self):
        if self._fake_surface:
            self._surface = None
            self._fake_surface = False
        # return True

class FeatureFromNormals(Feature, metaclass=abc.ABCMeta):
    '''
    Feature with normals as input
    '''
    def __init__(self, cloud=None, indices=None, normals=None):
        super().__init__(cloud, indices)
        self._normals = normals

    @property
    def input_normals(self):
        '''
        Gets the normals of the input XYZ point cloud dataset.
        '''
        return self._normals

    @input_normals.setter
    def input_normals(self, value):
        '''
        Sets the normals of the input XYZ point cloud dataset

        In case of search surface is set to be different from the input cloud,
        normals should correspond to the search surface, not the input cloud!

        By convention, L2 norm of each normal should be 1.
        '''
        if not value or not ('normal_x' in value.names and 'curvature' in value.names):
            raise ValueError('invalid input normals cloud')
        self._normals = value

    def _init_compute(self):
        if not super()._init_compute():
            return False

        if self._normals is None:
            raise ValueError('no input dataset containing normals was given')

        if len(self._normals) != len(self._surface):
            msg1 = 'the number of points in the input dataset (%u) differs from '
            msg2 = 'the numberof points in the dataset containing the normals (%u)'
            raise ValueError(msg1 % len(self._surface) + msg2 % len(self._normals))

        return True

class FeatureFromLabels(Feature, metaclass=abc.ABCMeta):
    '''
    Feature with labels as input
    '''
    def __init__(self, cloud=None, indices=None, labels=None):
        super().__init__(cloud, indices)
        self._labels = labels

    @property
    def input_labels(self):
        '''
        Gets the labels of the input XYZ point cloud dataset.
        '''
        return self._labels

    @input_labels.setter
    def input_labels(self, value):
        '''
        Sets the normals of the input XYZ point cloud dataset

        In case of search surface is set to be different from the input cloud,
        labels should correspond to the search surface, not the input cloud!
        '''
        if not value or 'label' not in value.names:
            raise ValueError('invalid input label cloud')
        self._labels = value

    def _init_compute(self):
        if not super()._init_compute():
            return False

        if self._labels is None:
            raise ValueError('no input dataset containing labels was given')

        if len(self._labels) != len(self._surface):
            msg1 = 'the number of points in the input dataset (%u) differs from '
            msg2 = 'the numberof points in the dataset containing the labels (%u)'
            raise ValueError(msg1 % len(self._surface) + msg2 % len(self._labels))

        return True

class FeatureWithLocalReferenceFrames(Feature, metaclass=abc.ABCMeta):
    '''
    FeatureWithLocalReferenceFrames provides a public interface for descriptor
    extractor classes which need a local reference frame at each input keypoint.

    This interface is for backward compatibility with existing code and in the future it could be
    merged with pcl.features.Feature. Subclasses should call the protected method
    init_local_reference_frames () to correctly initialize the _frames member.
    '''
    pass

Feature.register(FeatureFromNormals)
Feature.register(FeatureFromLabels)
