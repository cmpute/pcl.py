from pcl.PointField import *
from pcl.PointCloud import *

from pcl.common import *
from pcl.filters import *
from pcl.io import *
from pcl.visualization import *

def get_include():
    import pcl, os
    return os.path.join(os.path.dirname(pcl.__file__), 'include')

############################ Helper Functions #############################

import numpy as np

def _check_point_size(data, size):
    data = np.array(data, copy=False)
    if data.shape[-1] != size:
        raise ValueError("Each point should contain %d values" % size)
    return data

def _parse_rgb(data, offset):
    rgb_dt = np.dtype(dict(names=['b','g','r'], formats=['u1','u1','u1'], itemsize=4))
    rgb_arr = np.empty(len(data), dtype=rgb_dt)
    rgb_arr['r'] = data[:,offset].copy() # copy to prevent double free problem
    rgb_arr['g'] = data[:,offset+1].copy()
    rgb_arr['b'] = data[:,offset+2].copy()
    return rgb_arr.view('u4')

def create_xyz(data):
    '''
    Create PointXYZ point cloud from ordinary nx3 array, or equivalent list representation
    '''
    data = _check_point_size(data, 3)

    dt = np.dtype(dict(names=['x','y','z'], formats=['f4','f4','f4'], itemsize=16))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'] = data[:,0], data[:,1], data[:,2]
    return PointCloud(arr, 'XYZ')

def create_rgb(data):
    '''
    Create RGB point cloud from ordinary nx3 array, or equivalent list representation.
    Note that RGB value should be represented in integer of 0~255
    '''
    data = _check_point_size(data, 3)
    return PointCloud(_parse_rgb(data, 0), 'RGB')

def create_xyzrgb(data):
    '''
    Create PointXYZRGB point cloud from ordinary nx6 array, or equivalent list representation
    Note that RGB value should be represented in integer of 0~255
    '''
    data = _check_point_size(data, 6)

    dt = np.dtype(dict(names=['x','y','z','rgb'], formats=['f4','f4','f4','u4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['rgb'] = data[:,0], data[:,1], data[:,2], _parse_rgb(data, 3)
    return PointCloud(arr, 'XYZRGB')

def create_xyzrgba(data):
    '''
    Create PointXYZRGBA point cloud from ordinary nx6 array, or equivalent list representation
    '''
    data = _check_point_size(data, 7)

    rgba_dt = np.dtype(dict(names=['b','g','r','a'], formats=['u1','u1','u1','u1'], itemsize=4))
    rgba_arr = np.empty(len(data), dtype=rgba_dt)
    rgba_arr['r'], rgba_arr['g'], rgba_arr['b'], rgba_arr['a'] = data[:,3], data[:,4], data[:,5], data[:,6]

    dt = np.dtype(dict(names=['x','y','z','rgba'], formats=['f4','f4','f4','u4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['rgba'] = data[:,0], data[:,1], data[:,2], rgba_arr.view('u4')
    return PointCloud(arr, 'XYZRGBA')

def create_xyzi(data):
    '''
    Create PointXYZI point cloud from ordinary nx4 array, or equivalent list representation
    '''
    data = _check_point_size(data, 4)

    dt = np.dtype(dict(names=['x','y','z','intensity'], formats=['f4','f4','f4','f4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['intensity'] = data[:,0], data[:,1], data[:,2], data[:,3]
    return PointCloud(arr, 'XYZI')

def create_normal(data):
    '''
    Create Normal (point type) point cloud from ordinary nx3 or nx4 array, or equivalent list representation
    '''
    data = np.array(data, copy=False)
    if data.shape[-1] not in [3, 4]:
        raise ValueError("Each point should contain 3 or 4 values")

    dt = np.dtype(dict(names=['normal_x','normal_y','normal_z','curvature'], formats=['f4','f4','f4','f4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['normal_x'], arr['normal_y'], arr['normal_z'] = data[:,0], data[:,1], data[:,2]
    if data.shape[-1] == 4:
        arr['curvature'] = data[:,3]
    else:
        arr['curvature'] = 0
    return PointCloud(arr, 'NORMAL')

def create_xyzl(data):
    data = _check_point_size(data, 4)

    dt = np.dtype(dict(names=['x','y','z','label'], formats=['f4','f4','f4','u4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['label'] = data[:,0], data[:,1], data[:,2], data[:,3]
    return PointCloud(arr, 'XYZL')

def create_xyzrgbl(data):
    data = _check_point_size(data, 7)

    dt = np.dtype(dict(names=['x','y','z','rgb','label'], formats=['f4','f4','f4','u4','u4'], offsets=[0,4,8,16,20], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['rgb'], arr['label'] = data[:,0], data[:,1], data[:,2], _parse_rgb(data, 3), data[:,6]
    return PointCloud(arr, 'XYZRGBL')
