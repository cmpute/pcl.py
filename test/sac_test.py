'''
Tests of pcl.sac
'''

import os
import sys
import logging
import numpy as np
from numpy.random import RandomState
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.sac as ps

def test_ransac():
    '''
    Test RandomSampleConsensus
    '''
    num = 1000
    rng = RandomState()
    scale = 10
    points = np.zeros((num, 3), 'f8')
    for i in range(num):
        points[i, 0] = rng.rand() * scale
        points[i, 1] = rng.rand() * scale
        if i % 2 == 0:
            points[i, 2] = -points[i, 0] - points[i, 1]
        else:
            points[i, 2] = rng.rand() * scale
    cloud = pcl.PointCloud(points, fields=['x', 'y', 'z'])
    ransac = ps.RandomSampleConsensus(ps.SampleConsensusModelPlane(cloud))
    assert len(ransac.get_random_samples(np.random.choice(num, 50), 30)) == 30
    ransac.max_iterations = 1000
    ransac.distance_threshold = 0.1
    ransac.compute_model()
    assert len(ransac.model_coefficients) == 4
    assert len(ransac.inliers) > 0
    assert len(ransac.model) == 3
    ransac.refine_model()
    assert len(ransac.model_coefficients) == 4
    assert len(ransac.inliers) > 0
    assert len(ransac.model) == 3

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
