'''
Tests of pcl.search
'''

import os
import sys
import numpy as np
from numpy.linalg import norm
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.search as ps

def test_brute_force():
    '''
    Test BruteForceSearch
    '''
    seq = np.arange(120).reshape(-1, 3)
    cloud = pcl.PointCloud(seq, ['x', 'y', 'z'])
    search = ps.BruteForceSearch(cloud, sort_results=True)

    # kNN
    query = [20, 20, 20]
    num = 5
    indices, distance = search.nearestk_search(query, num)
    assert (norm(cloud.xyz[indices] - query, axis=1) == distance).all()
    assert (indices == [6, 7, 5, 8, 4]).all()
    query = [50, 50, 50]
    indices, distance = search.nearestk_search(query, num)
    assert (norm(cloud.xyz[indices] - query, axis=1) == distance).all()
    assert (indices == [16, 17, 15, 18, 14]).all()
    search.sort_results = False
    indices2, distance2 = search.nearestk_search(query, num)
    assert not set(indices).difference(set(indices2))
    assert not set(distance).difference(set(distance2))

    #radius Search
    query = [20, 20, 20]
    radius = 5
    indices, distance = search.radius_search(query, radius)
    assert (norm(cloud.xyz[indices] - query, axis=1) <= radius).all()
    assert (indices == [6, 7]).all()
    del cloud[indices]
    assert (norm(cloud.xyz - query, axis=1) > radius).all()

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
