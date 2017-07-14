'''
Tests of pcl.segment.normal
'''

import os, sys
import numpy as np
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.features as pf

def test_normal():
    '''
    Test NormalEstimation
    '''
    # cloud = pcl.io.loadpcd(os.path.dirname(__file__) + '/data/car6.pcd')
    cloud = pcl.PointCloud(np.random.rand(20, 3), ['x', 'y', 'z'])
    nestimate = pf.NormalEstimation(cloud)
    nestimate.search_k = 5
    normalcloud = nestimate.compute()
    assert len(normalcloud) == len(cloud)

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
# test_normal()
