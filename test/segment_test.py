'''
Tests of pcl.segment
'''

import os
import sys
import logging
import numpy as np
from numpy.random import RandomState
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.segment as ps
import pcl.features as pf

def test_regiongrow():
    '''
    Test RegionGrowing
    '''

    cloud = pcl.PointCloud(np.random.rand(200, 3), ['x', 'y', 'z'])
    # cloud = pcl.io.loadpcd(os.path.dirname(__file__) + '/data/car6.pcd')
    regiongrow = ps.RegionGrowing(cloud)
    with pytest.raises(ValueError): # No normal is inputted
        regiongrow.extract()
    nestimate = pf.NormalEstimation(cloud)
    nestimate.search_k = 10
    normals = nestimate.compute()
    regiongrow.input_normals = normals
    regions = regiongrow.extract()
    # TODO: visual or other checking is needed

    cluster = regions[-1]
    assert regiongrow.get_segment_from_point(cluster[-1]) == cluster
    color = regiongrow.get_colored_cloud()
    colora = regiongrow.get_colored_cloud_rgba()
    assert (colora.rgba['a'] == 255).all()
    coloredcloud = cloud + color
    coloredclouda = cloud + colora
    assert 'rgb' in coloredcloud.names
    assert 'rgba' in coloredclouda.names

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
