'''
Tests of pcl.pointcloud
'''

import os
import sys
import pickle
import pytest
import numpy as np
from io import BytesIO
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl

def test_build_cloud():
    '''
    Test Generating point cloud from arrays
    '''
    cloud = pcl.PointCloud()
    cloud = pcl.PointCloud(width=2, height=2)
    with pytest.raises(TypeError):
        cloud = pcl.PointCloud(fields=('x', 'y', 'z'))
        cloud = pcl.PointCloud(width=2, height=2, fields=('x', 'y', 'z'))
    assert cloud.fields == ()

    corddata = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    cloud = pcl.PointCloud(corddata, ['x', 'y', 'z'])
    assert cloud.xyz.dtype == np.dtype(int)
    assert cloud.xyz.tolist() == corddata

    data = [[1, 2., 3.], [4, 5., 6.]]
    cloud = pcl.PointCloud(data, ('x', 'y', 'z'))
    cloud = pcl.PointCloud(data, (('x', 'i1'), ('y', 'f2'), ('z', 'f2')))
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))
    cloud = pcl.PointCloud(data, [('x', 1, 'I'), ('y', 'f', 2), ('z', 2, 'F')])
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))
    cloud = pcl.PointCloud(data, cloud.data.dtype)
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))

    data = [[1, (2., 3.)], [4, (5., 6.)]]
    # pity that numpy do not support inferring tuple types
    # cloud = pcl.PointCloud(data, ('x', 'yz'))
    cloud = pcl.PointCloud(data, (('x', 'i1'), ('yz', '2f2')))
    assert cloud.fields == (('x', 'i1'), ('yz', '2f2'))
    cloud = pcl.PointCloud(data, [('x', 1, 'I'), ('yz', 'f', 2, 2)])
    assert cloud.fields == (('x', 'i1'), ('yz', '2f2'))
    cloud = pcl.PointCloud(data, cloud.data.dtype)
    assert cloud.fields == (('x', 'i1'), ('yz', '2f2'))

    data = [[1, 2., 3.], [4, 5., 6.], [7, 8., 9.], [10, 11., 12.]]
    cloud = pcl.PointCloud(width=2, height=2, fields=(('x', 'i1'), ('y', 'f2'), ('z', 'f2')))
    assert cloud.is_organized
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))
    cloud = pcl.PointCloud(data, width=2, height=2, fields=('x', 'y', 'z'))
    cloud = pcl.PointCloud(data, width=2, height=2, fields=(('x', 'i1'), ('y', 'f2'), ('z', 'f2')))
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))
    cloud = pcl.PointCloud(data, width=2, height=2,
                           fields=[('x', 1, 'I'), ('y', 'f', 2), ('z', 2, 'F')])
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))
    cloud = pcl.PointCloud(cloud, cloud.data.dtype)
    assert cloud.fields == (('x', 'i1'), ('y', 'f2'), ('z', 'f2'))

    cloud.sensor_origin[:] = [1, 2, 3, 4]
    return cloud

def test_cloud_operations():
    '''
    Test operations on the point cloud
    '''
    cloud = test_build_cloud()
    assert cloud == pcl.PointCloud(cloud)
    assert cloud + cloud == pcl.PointCloud(cloud + cloud)

    # pickle support test
    src = BytesIO()
    pick = pickle.Pickler(src)
    pick.dump(cloud)
    src.seek(0)
    pick = pickle.Unpickler(src)
    assert cloud == pick.load()

    # customized operators test
    assert cloud.names == ['x', 'y', 'z']
    assert cloud.width == 2 and cloud.height == 2
    assert cloud.data.tolist() == [(1, 2., 3.), (4, 5., 6.), (7, 8., 9.), (10, 11., 12.)]
    assert cloud[2].data.tolist() == (7, 8., 9.)
    assert cloud[:2].data.tolist() == [(1, 2., 3.), (4, 5., 6.)]
    assert cloud[1, 0] == cloud[2] and cloud[0, 1] == cloud[1]
    assert cloud[:2, :2].data.tolist() == [[(1, 2., 3.), (4, 5., 6.)],
                                           [(7, 8., 9.), (10, 11., 12.)]]
    cloud[1, 0] = (5, 6., 7.)
    assert cloud.data.tolist() == [(1, 2., 3.), (4, 5., 6.), (5, 6., 7.), (10, 11., 12.)]
    cloud[:2, :2] = [(7, 8., 9.), (10, 11., 12.), (1, 2., 3.), (4, 5., 6.)]
    assert cloud.data.tolist() == [(7, 8., 9.), (10, 11., 12.), (1, 2., 3.), (4, 5., 6.)]
    cloud[:2, :2] = [[(1, 2., 3.), (4, 5., 6.)], [(7, 8., 9.), (10, 11., 12.)]]
    assert cloud.data.tolist() == [(1, 2., 3.), (4, 5., 6.), (7, 8., 9.), (10, 11., 12.)]
    del cloud[0, 0]
    assert not cloud.is_organized
    del cloud[[1, 3]]
    cloud.insert([1, 2], [(2, 3., 3.), (4, 6, 6)])
    cloud.append([(0, 0, 0)])
    assert cloud.pop().tolist() == (0, 0., 0.)
    cloud += [(10, 9., 8.), (8, 9., 10.)]
    assert len(cloud) == 6
    del cloud[:5]
    assert len(cloud) == 1

def test_cloud_field_operations():
    '''
    Test operations on the fields of point cloud
    '''
    cloud = test_build_cloud()

    assert cloud['x'].data.tolist() == [1, 4, 7, 10]
    assert cloud[['x', 'y']].data.tolist() == [(1, 2.), (4, 5.), (7, 8.), (10, 11.)]
    assert cloud['x', 'y'].data.tolist() == [(1, 2.), (4, 5.), (7, 8.), (10, 11.)]
    with pytest.raises(TypeError):
        cloud.append_fields([('x', 'i2')], 4)
        cloud.insert_fields([('x', 'i1')], [3])
    assert cloud.to_ndarray(['y', 'z']).tolist() == [[2, 3], [5, 6], [8, 9], [11, 12]]

    cloud.append_fields([('w', 'i2')], 4)
    assert cloud.names == ['x', 'y', 'z', 'w']
    assert cloud['w'].data.tolist() == [4, 4, 4, 4]
    cloud.append_fields([('a', 'f2'), ('b', 'f2')], {'a':[1, 2, 3, 4], 'b':[5, 6, 7, 8]})
    assert cloud.names == ['x', 'y', 'z', 'w', 'a', 'b']
    cloud.insert_fields([('t', 'i1')], [3])
    assert cloud.names == ['x', 'y', 'z', 't', 'w', 'a', 'b']
    tdata = cloud.data.copy()
    cloud['x', 'y'] = cloud['y', 'x']
    assert (tdata == cloud.data).all() # Note: the setter gets data by fields, not by order
    cloud[['x', 'y']] = cloud['y', 'x']
    assert (tdata == cloud.data).all()
    del cloud['w']
    del cloud['a', 'b']
    assert cloud.names == ['x', 'y', 'z', 't']
    cloud.insert_fields([('r', 'i1'), ('s', 'i2'), ('u', 'f8')], (3, 3, 4), \
                        {'r':[0, 0, 0, 0], 's':[1, 1, 1, 1]})
    assert cloud.names == ['x', 'y', 'z', 'r', 's', 't', 'u']
    del cloud[['x', 'y', 'z', 'u']]
    assert cloud.names == ['r', 's', 't']
    cloud.insert_fields([('k', 'f8')], [3], 6)
    cloud.insert_fields([('a', 'i2'), ('b', 'i4')], [4, 4], \
                        [[1, 2, 3, 4], [5, 6, 7, 8]])
    del cloud['a', 'b']
    cloud.insert_fields([('a', 'i2'), ('b', 'i4')], [4, 4], \
                        np.array([(1, 5), (2, 6), (3, 7), (4, 8)], \
                                     dtype=[('a', 'i2'), ('b', 'i4')]))
    assert cloud.names == ['r', 's', 't', 'k', 'a', 'b']

    cloud2 = pcl.PointCloud(cloud.pop_fields(['r', 't', 'k']), fields=['r', 't', 'k'])
    cloud.copy_metadata(cloud2)
    assert (cloud + cloud2).names == ['s', 'a', 'b', 'r', 't', 'k']
    cloud.rename_fields('s1', old_names='s')
    assert cloud.names == ['s1', 'a', 'b']
    cloud.rename_fields(['s2', 'b1'], [0, 2])
    assert cloud.names == ['s2', 'a', 'b1']
    cloud.rename_fields(['a1', 'b2'], old_names=['a', 'b1'])
    assert cloud.names == ['s2', 'a1', 'b2']
    cloud.rename_fields(['a', 'b', 's'], offsets=[1, 2, 0], old_names=['a1', 'b2', 's2'])

    # single field test
    cloud = test_build_cloud()
    del cloud['y', 'z']
    cloud.insert_fields([('r', 'i1'), ('s', 'i2')], (0, 1), {'r':[0, 0, 0, 0], 's':[1, 1, 1, 1]})
    assert cloud.data.dtype.names == ('r', 'x', 's')
    del cloud['r', 's']
    cloud.append_fields([('w', 'i2')], 4)
    assert cloud.data.dtype.names == ('x', 'w')
    assert cloud['w'].data.tolist() == [4, 4, 4, 4]
    del cloud[:3]
    with pytest.raises(TypeError):
        cloud.append_fields([('w', 'i2')], 4)
        cloud.insert_fields([('w', 'i2')], 4)
    cloud.append_fields([('r', 'i2')], 4)
    del cloud['w', 'r']
    cloud += cloud
    assert cloud['x'].data.tolist() == [10, 10]

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
# test_cloud_operations()
