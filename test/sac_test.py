'''
Tests of pcl.sac
'''

import os, sys
import numpy as np
from numpy.random import RandomState
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.sac as ps

def test_model_plane():
    '''
    Test SampleConsensusModelPlane
    '''
    num = 1000
    rng = RandomState(12345)
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
    plane = ps.SampleConsensusModelPlane(cloud, random=True)
    assert plane.model_size == 4
    assert plane.sample_size == 3
    samples = plane.get_samples()
    _, coeff = plane.compute_model_coefficients(samples)
    assert len(coeff) == plane.model_size
    assert (plane.get_distance_to_model(coeff) >= 0).all()
    inliers = plane.select_within_distance(coeff, 1)
    assert len(inliers) == plane.count_within_distance(coeff, 1) >= 0
    optcoeff = plane.optimize_model_coefficients(inliers, coeff)
    assert np.mean(plane.get_distance_to_model(optcoeff)) <=\
           np.mean(plane.get_distance_to_model(coeff))
    project = plane.project_points(inliers, optcoeff)
    assert (np.dot(project.xyz, optcoeff[:3]) + optcoeff[3] < 1e-7).all()

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
    assert len(ransac.get_random_samples(rng.choice(num, 50), 30)) == 30
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
