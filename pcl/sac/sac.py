'''
Implementation of following files:
    pcl/sample_consensus/include/pcl/sample_consensus/sac.h
'''

import abc
import logging
import math
import numpy as np
from numpy.random import RandomState

class SampleConsensus(metaclass=abc.ABCMeta):
    '''
    SampleConsensus represents the base class.
    All sample consensus methods must inherit from this class.
    '''

    def __init__(self, model,
                 random=False,
                 probability=.99,
                 threshold=float('inf'), max_iterations=1000):
        self._sac_model = model
        self.probability = probability
        self.distance_threshold = threshold
        self.max_iterations = max_iterations
        self._model = []
        self._inliers = []
        self._model_coefficients = None
        if random:
            self._rng = RandomState()
        else:
            self._rng = RandomState(12345)

    @abc.abstractmethod
    def compute_model(self):
        '''
        Compute the actual model. Pure virtual.
        '''
        pass

    @property
    def model(self):
        '''
        Return the best model found so far.
        '''
        return self._model

    @property
    def inliers(self):
        '''
        Return the best set of inliers found so far for this model.
        '''
        return self._inliers

    @property
    def model_coefficients(self):
        '''
        Return the model coefficients of the best model found so far.
        '''
        return self._model_coefficients

    @property
    def sample_consensus_model(self):
        '''
        Get the Sample Consensus model used
        '''
        return self._sac_model

    @sample_consensus_model.setter
    def sample_consensus_model(self, value):
        '''
        Set the Sample Consensus model to use
        '''
        self._sac_model = value

    def refine_model(self, sigma=3., max_iterations=1000):
        '''
        Refine the model found.

        This loops over the model coefficients and optimizes them together
        with the set of inliers, until the change in the set of inliers is
        minimal.

        # Parameters
        sigma : float
            standard deviation multiplier for considering a sample as inlier (Mahalanobis distance)
        max_iterations : int
            the maxim number of iterations to try to refine in case the inliers keep on changing
        '''
        if self._sac_model is None:
            raise ValueError('null model!')
        logger = logging.getLogger('pcl.sac.SampleConsensus.refineModel')

        inlier_distance_threshold_sqr = self.distance_threshold * self.distance_threshold
        error_threshold = self.distance_threshold
        sigma_sqr = sigma * sigma
        refine_iterations = 0
        inlier_changed, oscillating = False, False
        inliers_sizes = []
        new_inliers, prev_inliers = [], []
        new_model_coefficients = None
        while True:
            # Optimize the model coefficients
            new_model_coefficients = self._sac_model\
                                         .optimize_model_coefficients(prev_inliers,
                                                                      new_model_coefficients)
            inliers_sizes.append(len(prev_inliers))

            # Select the new inliers based on the optimized coefficients and new threshold
            self._sac_model.select_within_distance(new_model_coefficients,
                                                   error_threshold,
                                                   new_inliers)
            logger.debug('Number of inliers found (before/after): %lu/%lu, ' +
                         'with an error threshold of %g.',
                         len(prev_inliers), len(new_inliers), error_threshold)

            if len(new_inliers) == 0:
                refine_iterations += 1
                if refine_iterations >= max_iterations:
                    break
                continue

            # Estimate the variance and the new threshold
            variance = self._sac_model.compute_variance()
            error_threshold = math.sqrt(min(inlier_distance_threshold_sqr, sigma_sqr * variance))

            logger.debug('New estimated error threshold: %g on iteration %d out of %d.',
                         error_threshold, refine_iterations, max_iterations)
            inlier_changed = False
            prev_inliers, new_inliers = new_inliers, prev_inliers
            # If the number of inliers changed, then we are still optimizing
            if len(new_inliers) > len(prev_inliers):
                # Check if the number of inliers is oscillating in between two values
                if len(inliers_sizes) >= 4:
                    if inliers_sizes[-1] == inliers_sizes[-3] and \
                       inliers_sizes[-2] == inliers_sizes[-4]:
                        oscillating = False
                        break
                inlier_changed = True
                continue

            # Check the values of the inlier set
            for idx, val in enumerate(prev_inliers):
                # If the value of the inliers changed, then we are still optimizing
                if val != new_inliers[idx]:
                    inlier_changed = True
                    break

            refine_iterations += 1
            if inlier_changed and refine_iterations < max_iterations:
                break

            # If the new set of inliers is empty, we didn't do a good job refining
            if len(new_inliers) == 0:
                logger.error('Refinement failed: got an empty set of inliers!')
                return False

            if oscillating:
                logger.debug('Detected oscillations in the model refinement.')
                return True

            if not inlier_changed:
                self._inliers, new_inliers = new_inliers, self._inliers
                self._model_coefficients = new_model_coefficients
                return True

            # If no inliers have been changed anymore, then the refinement was successful
            return False

    def get_random_samples(self, indices, nr_samples):
        '''
        Get a set of randomly selected indices.

        # Parameters
            indices : list or array
                The input indices vector
            nr_samples : int
                The desired number of point indices to randomly select
        '''
        sample = self._rng.random_sample(nr_samples) * len(indices)
        return np.array(indices, copy=False)[sample.astype(int)]

    def _rnd(self):
        '''
        Generate random number
        '''
        return self._rng.rand()

class RandomSampleConsensus(SampleConsensus):
    '''
    RandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus)
    algorithm, as described in: "Random Sample Consensus: A Paradigm for Model Fitting with
    Applications to Image Analysis and Automated Cartography",
    Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June 1981.
    '''
    pass

class RandomizedRandomSampleConsensus(SampleConsensus):
    '''
    RandomizedRandomSampleConsensus represents an implementation of the RRANSAC (Randomized
    RAndom SAmple Consensus), as described in "Randomized RANSAC with Td,d test", O. Chum and
    J. Matas, Proc. British Machine Vision Conf. (BMVC '02), vol. 2, BMVA, pp. 448-457, 2002.

    RRANSAC is useful in situations where most of the data samples belong to the model,
    and a fast outlier rejection algorithm is needed.
    '''
    pass

class MEstimatorSampleConsensus(SampleConsensus):
    '''
    MEstimatorSampleConsensus represents an implementation of the MSAC (M-estimator SAmple
    Consensus) algorithm, as described in: "MLESAC: A new robust estimator with application
    to estimating image geometry", P.H.S. Torr and A. Zisserman,
    Computer Vision and Image Understanding, vol 78, 2000.
    '''
    pass

class RandomizedEstimatorSampleConsensus(SampleConsensus):
    '''
    RandomizedMEstimatorSampleConsensus represents an implementation of the RMSAC
    (Randomized M-estimator SAmple Consensus) algorithm, which basically adds a Td,d test
    (see RandomizedRandomSampleConsensus) to an MSAC estimator (see MEstimatorSampleConsensus).

    RMSAC is useful in situations where most of the data samples belong to the model,
    and a fast outlier rejection algorithm is needed.
    '''
    pass

class ProgressiveSampleConsensus(SampleConsensus):
    '''
    RandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus)
    algorithm, as described in: "Matching with PROSAC – Progressive Sample Consensus",
    Chum, O. and Matas, J.G., CVPR, I: 220-226 2005.
    '''
    pass

class LeastMedianSquares(SampleConsensus):
    '''
    LeastMedianSquares represents an implementation of the LMedS (Least Median of Squares)
    algorithm. LMedS is a RANSAC-like model-fitting algorithm that can tolerate up to 50% outliers
    without requiring thresholds to be set. See Andrea Fusiello's "Elements of Geometric
    Computer Vision"
    (http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO4/tutorial.html#x1-520007)
    for more details.
    '''
    pass

class MaximumLikelihoodSampleConsensus(SampleConsensus):
    '''
    MaximumLikelihoodSampleConsensus represents an implementation of the MLESAC (Maximum
    Likelihood Estimator SAmple Consensus) algorithm, as described in: "MLESAC: A new robust
    estimator with application to estimating image geometry", P.H.S. Torr and A. Zisserman,
    Computer Vision and Image Understanding, vol 78, 2000.

    MLESAC is useful in situations where most of the data samples belong to the model,
    and a fast outlier rejection algorithm is needed.
    '''
    pass


# alias
RANSAC = RandomSampleConsensus
RRANSAC = RandomizedRandomSampleConsensus
MSAC = MEstimatorSampleConsensus
RMSAC = RandomizedEstimatorSampleConsensus
PROSAC = ProgressiveSampleConsensus
LMedS = LeastMedianSquares
MLESAC = MaximumLikelihoodSampleConsensus
