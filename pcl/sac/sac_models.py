'''
Implementation of following files:
    pcl/sample_consensus/include/pcl/sample_consensus/sac_model.h
    pcl/sample_consensus/include/pcl/sample_consensus/sac_model_plane.h
    pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp
	pcl/sample_consensus/src/sac_model_plane.cpp

Following files are abandoned:
    pcl/sample_consensus/include/pcl/sample_consensus/model_types.h
'''

import abc
import logging
import numpy as np
from numpy.linalg import norm, lstsq
from numpy.random import RandomState
from ..common import _CloudBase
from ..pointcloud import PointCloud

class SampleConsensusModel(_CloudBase, metaclass=abc.ABCMeta):
    '''
    SampleConsensusModel represents the base model class. All sample consensus models must inherit
    from this class.
    '''
    def __init__(self, cloud, indices=None, random=False):
        super().__init__(cloud, indices=None)
        self._radius_limits = (-float('inf'), float('inf')) # use tuple to represent interval
        # XXX: Whether it should be a private member
        self.samples_max_dist = 0 # samples_radius_
        self._samples_radius_search = None
        self._error_sqr_dists = []
        # The maximum number of samples to try until we get a good one
        self._max_sample_checks = 1000

        # essential fields
        self._sample_size = 0
        self._model_size = 0

        if random:
            self._rng = RandomState()
        else:
            self._rng = RandomState(12345)

    @property
    def sample_size(self):
        '''
        The size of a sample from which the model is computed.
        '''
        return self._sample_size

    @property
    def model_size(self):
        '''
        The number of coefficients in the model.
        '''
        return self._model_size

    @property
    def radius_limits(self):
        '''
        Get the minimum and maximum allowable radius limits for the model as set by the user.

        # Returns
        tuple (min_radius, max_radius)
        '''
        return self._radius_limits

    @radius_limits.setter
    def radius_limits(self, value):
        '''
        Set the maximum distance allowed when drawing random samples
        '''
        try:
            min_radius, max_radius = value
            min_radius = float(min_radius)
            max_radius = float(max_radius)
        except:
            raise ValueError('the radius limits should be a tuple of (min_radius, max_radius)')
        self._radius_limits = (min_radius, max_radius)

    def get_samples(self):
        '''
        Get a set of random data samples and return them as point indices.

        # Returns
        samples : list of int
            the resultant model samples
        '''
        logger = logging.getLogger('pcl.sac.SampleConsensusModel.get_samples')
        if len(self._indices) < self.sample_size:
            logger.error('Can not select %lu unique points out of %lu'
                         , len(self._indices), self.sample_size)
            return []

        for _ in range(self._max_sample_checks):
            if self.samples_max_dist < np.finfo(float).eps:
                samples = self._draw_index_sample()
            else:
                samples = self._draw_index_sample_radius()

            if self._is_sample_good(samples):
                logger.debug('Selected %lu samples', len(samples))
                return samples

        logger.warning('Could not select %d sample points in %d iterations!'
                       , self.sample_size, self._max_sample_checks)
        return []

    @abc.abstractmethod
    def compute_model_coefficients(self, samples):
        '''
        Check whether the given index samples can form a valid model, compute the model
        coefficients from these samples and store them in model_coefficients.

        # Parameters
        samples : list of int
            The point indices found as possible good candidates for creating a valid model

        # Returns
        success : bool
            True if no error occurs in the iteration
        model_coefficients : coefficients array
            The computed model coefficients
        '''
        pass

    @abc.abstractmethod
    def optimize_model_coefficients(self, inliers, model_coefficients):
        '''
        Recompute the model coefficients using the given inlier set and return them to the user.

        These are the coefficients of the model after refinement
        (e.g., after a least-squares optimization)

        # Parameters
        inliers : list of int
            The data inliers supporting the model
        model_coefficients : coefficients array
            The initial guess for the model coefficients

        # Returns
        optimized_coefficients : coefficients array
            The resultant recomputed coefficients after non-linear optimization
        '''
        pass

    @abc.abstractmethod
    def get_distance_to_model(self, model_coefficients):
        '''
        Compute all distances from the cloud data to a given model.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to

        # Returns
        distances : list of float
            The resultant estimated distances
        '''
        pass

    @abc.abstractmethod
    def select_within_distance(self, model_coefficients, threshold):
        '''
        Select all the points which respect the given model coefficients as inliers.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        inliers : list of int
            The resultant model inliers
        '''
        pass

    @abc.abstractmethod
    def count_within_distance(self, model_coefficients, threshold):
        '''
        Count all the points which respect the given model coefficients as inliers.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        count : int
            The resultant number of inliers
        '''
        pass

    @abc.abstractmethod
    def project_points(self, inliers, model_coefficients, copy_data_fields=True):
        '''
        Create a new point cloud with inliers projected onto the model.

        # Parameters
        inliers : list of int
            The data inliers that we want to project on the model
        model_coefficients : coefficients array
            The coefficients of a model
        copy_data_fields : bool
            Set to true (default) if we want the projected_points cloud to be an exact copy of the
            input dataset minus the point projections on the plane model

        # Returns
        projected_points : PointCloud
            The resultant projected points
        '''
        pass

    @abc.abstractmethod
    def do_sample_verify_model(self, indices, model_coefficients, threshold):
        '''
        Verify whether a subset of indices verifies a given set of model coefficients.

        # Parameters
        indices : list of int
            the data indices that need to be tested against the model
        model_coefficients : coefficients array
            the set of model coefficients
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        result : bool
        '''
        pass

    def compute_variance(self, error_sqr_dists=None):
        '''
        Compute the variance of the errors to the model.

        # Parameters
        error_sqr_dists : list of float
            A vector holding the distances

        # Returns
        variance : double
        '''
        if error_sqr_dists is None:
            if len(self._error_sqr_dists) == 0:
                raise ValueError('The variance of the Sample Consensus model distances cannot \
                be estimated, as the model has not been computed yet. Please compute the model \
                first or at least run selectWithinDistance before continuing.')
            error_sqr_dists = self._error_sqr_dists

        medidx = int(len(error_sqr_dists) / 2)
        return 2.1981 * np.partition(error_sqr_dists, medidx)[medidx]

    def _draw_index_sample(self):
        '''
        Fills a sample array with random samples from the _indices vector

        # Returns
        sample : list of int
            sample the set of indices of target_ to analyze
        '''
        return self._rng.choice(self._indices, self.sample_size)

    def _draw_index_sample_radius(self):
        '''
        Fills a sample array with one random sample from the indices_ vector
        and other random samples that are closer than _samples_radius

        # Returns
        sample : list of int
            sample the set of indices of target_ to analyze
        '''
        randfirst = self._input[self._indices[self._rng.randint(self._indices)]]
        indices, _ = self._samples_radius_search.radius_search(randfirst, self.samples_max_dist)
        if len(indices) < self.sample_size - 1:
            # radius search failed, make an invalid model
            return np.tile(randfirst, (self.sample_size, 1))
        else:
            return self._rng.choice(indices, self.sample_size)

    def _is_model_valid(self, model_coefficients):
        '''
        Check whether a model is valid given the user constraints.

        Default implementation verifies that the number of coefficients in the supplied model is
        as expected for this SAC model type. Specific SAC models should extend this function by
        checking the user constraints (if any).
        '''
        if len(model_coefficients) != self.model_size:
            return False
        return True

    @abc.abstractmethod
    def _is_sample_good(self, samples):
        '''
        Check if a sample of indices results in a good sample of points indices.

        # Parameters
        samples : list of int
            The indices of query samples
        '''
        pass

class SampleConsensusModelPlane(SampleConsensusModel):
    '''
    SampleConsensusModelPlane defines a model for 3D plane segmentation.

    # Model Coefficients
    a : the X coordinate of the plane's normal (normalized)
    b : the Y coordinate of the plane's normal (normalized)
    c : the Z coordinate of the plane's normal (normalized)
    d : the fourth Hessian component of the plane's equation
    '''
    def __init__(self, cloud, indices=None, random=False):
        super().__init__(cloud, indices, random)
        self._sample_size = 3
        self._model_size = 4

    def _is_sample_good(self, samples):
        '''
        Check if a sample of indices results in a good sample of points indices.

        # Parameters
        samples : list of int
            The indices of query samples
        '''
        if len(samples) < 3:
            return False
        cloud = self._input.xyz[samples]
        dy1dy2 = (cloud[1] - cloud[0]) / (cloud[2] - cloud[0])
        # check colinearity of the three points
        return dy1dy2[0] != dy1dy2[1] or dy1dy2[1] != dy1dy2[2]

    def compute_model_coefficients(self, samples):
        '''
        Check whether the given index samples can form a valid plane model, compute the model
        coefficients from these samples and store them internally in model_coefficients_.
        The plane coefficients are: a, b, c, d (ax+by+cz+d=0)

        # Parameters
        samples : list of int
            The point indices found as possible good candidates for creating a valid model

        # Returns
        success : bool
            True if no error occurs in the iteration
        model_coefficients : coefficients array
            The computed model coefficients
        '''
        if not self._is_sample_good(samples):
            return False, None

        cloud = self._input.xyz[samples]
        p1p0 = cloud[1] - cloud[0]
        p2p0 = cloud[2] - cloud[0]

        model_coefficients = [p1p0[1] * p2p0[0] - p1p0[2] * p2p0[1],
                              p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2],
                              p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0]]
        model_coefficients = np.array(model_coefficients) / norm(model_coefficients)
        return True, np.append(model_coefficients, -np.dot(cloud[0], model_coefficients))

    def get_distance_to_model(self, model_coefficients):
        '''
        Compute all distances from the cloud data to a given model.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to

        # Returns
        distances : list of float
            The resultant estimated distances
        '''
        if len(model_coefficients) != self.model_size:
            raise ValueError('Invalid number of model coefficients given.')

        points = self._input.xyz[self._indices]
        # points = np.append(points, np.ones(len(points)), axis=1)
        return np.abs(np.dot(points, model_coefficients[:3]) + model_coefficients[3])

    def select_within_distance(self, model_coefficients, threshold):
        '''
        Select all the points which respect the given model coefficients as inliers.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        inliers : list of int
            The resultant model inliers
        '''
        distance = self.get_distance_to_model(model_coefficients)
        predicate = distance < threshold
        self._error_sqr_dists = distance[predicate]
        return np.array(self._indices, copy=False)[predicate]

    def count_within_distance(self, model_coefficients, threshold):
        '''
        Count all the points which respect the given model coefficients as inliers.

        # Parameters
        model_coefficients : coefficients array
            The coefficients of a model that we need to compute distances to
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        count : int
            The resultant number of inliers
        '''
        distance = self.get_distance_to_model(model_coefficients)
        return np.sum(distance < threshold)

    def optimize_model_coefficients(self, inliers, model_coefficients):
        '''
        Recompute the model coefficients using the given inlier set and return them to the user.

        These are the coefficients of the model after refinement
        (e.g., after SVD)

        # Parameters
        inliers : list of int
            The data inliers supporting the model
        model_coefficients : coefficients array
            The initial guess for the model coefficients

        # Returns
        optimized_coefficients : coefficients array
            The resultant recomputed coefficients after non-linear optimization
        '''
        logger = logging.getLogger('pcl.sac.SampleConsensusModel.optimize_model_coefficients')
        if len(model_coefficients) != self.model_size:
            logger.error('Invalid number of model coefficients given (%lu).',
                         len(model_coefficients))
            return model_coefficients
        if len(inliers) <= self.sample_size:
            logger.error('Not enough inliers found to optimize model coefficients (%lu)!',
                         len(model_coefficients))
            return model_coefficients

        ###### Followings are implemetation of original PCL using PCA ######

        # covariance_matrix, xyz_centroid = compute_mean_and_covariance_matrix(self._input, inliers)
        # eigen_value, eigen_vector = eig(covariance_matrix)
        # smallest = np.argmin(eigen_value)
        # eigen_value = eigen_value[smallest]
        # eigen_vector = eigen_vector[:, smallest]

        # optimized_coefficients = eigen_vector.tolist()
        # optimized_coefficients.append(-np.dot(optimized_coefficients + [0], xyz_centroid))

        # Use Least-Squares to fit the plane through all the given sample points
        # and find out its coefficients

        cloud = self._input.xyz
        if inliers is not None:
            cloud = cloud[inliers]

        constant = -np.ones(len(cloud))
        optimized_coefficients, *_ = lstsq(cloud, constant)
        optimized_coefficients = optimized_coefficients.tolist()
        optimized_coefficients.append(1)

        if not self._is_model_valid(optimized_coefficients):
            logger.warning('Optimized coefficients invalid, returning original one')
            return model_coefficients

        return optimized_coefficients

    def project_points(self, inliers, model_coefficients, copy_data_fields=True):
        '''
        Create a new point cloud with inliers projected onto the model.

        # Parameters
        inliers : list of int
            The data inliers that we want to project on the model
        model_coefficients : coefficients array
            The coefficients of a model
        copy_data_fields : bool
            set to true if we need to copy the other data fields

        # Returns
        projected_points : PointCloud
            The resultant projected points
        '''
        if len(model_coefficients) != self.model_size:
            logger = logging.getLogger('pcl.sac.SampleConsensusModel.optimize_model_coefficients')
            logger.error('Invalid number of model coefficients given (%lu).',
                         len(model_coefficients))
            return None

        model_coefficients = np.array(model_coefficients, copy=False)
        model_coefficients = model_coefficients / norm(model_coefficients[:3])
        normvec = model_coefficients[:3]

        points = self._input.xyz[inliers]
        distance_to_plane = np.dot(points, normvec) + model_coefficients[3]
        project_points = points - normvec * distance_to_plane[:, np.newaxis]

        if copy_data_fields:
            cloud = PointCloud(self._input[inliers], fields=self._input.fields, copy=True)
            cloud['x'] = project_points[:, 0]
            cloud['y'] = project_points[:, 1]
            cloud['z'] = project_points[:, 2]
        else:
            cloud = PointCloud(project_points, fields=['x', 'y', 'z'], copy=False)

        return cloud

    def do_sample_verify_model(self, indices, model_coefficients, threshold):
        '''
        Verify whether a subset of indices verifies a given set of model coefficients.

        # Parameters
        indices : list of int
            The data indices that need to be tested against the model
        model_coefficients : coefficients array
            The plane model coefficients
        threshold : float
            A maximum admissible distance threshold for determining the inliers from the outliers

        # Returns
        result : bool
        '''
        if len(model_coefficients) != self.model_size:
            logger = logging.getLogger('pcl.sac.SampleConsensusModel.optimize_model_coefficients')
            logger.error('Invalid number of model coefficients given (%lu).',
                         len(model_coefficients))
            return False

        points = self._input.xyz[indices]
        distances = np.abs(np.dot(points, model_coefficients[:3]) + model_coefficients[3])
        return not (distances > threshold).any()

# alias
SACModel_Plane = SampleConsensusModelPlane
SampleConsensusModel.register(SACModel_Plane)
