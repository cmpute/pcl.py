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
    data = np.asarray(data)
    if data.shape[-1] != size:
        raise ValueError("Each point should contain %d values" % size)
    return data

def _parse_rgb(data, offset):
    rgb_dt = np.dtype(dict(names=['b','g','r'], formats=['u1','u1','u1'], itemsize=4))
    rgb_arr = np.empty(len(data), dtype=rgb_dt)
    rgb_arr['r'] = data[:,offset].copy() # FIXME: this copy() can prevent segfault in visualizer
    rgb_arr['g'] = data[:,offset+1]
    rgb_arr['b'] = data[:,offset+2]
    return rgb_arr.view('u4')

def _parse_rgba(data, offset):
    rgba_dt = np.dtype(dict(names=['b','g','r','a'], formats=['u1','u1','u1','u1'], itemsize=4))
    rgba_arr = np.empty(len(data), dtype=rgba_dt)
    rgba_arr['r'] = data[:,offset].copy() # FIXME: this copy() can prevent segfault in visualizer
    rgba_arr['g'] = data[:,offset+1]
    rgba_arr['b'] = data[:,offset+2]
    rgba_arr['a'] = data[:,offset+3]
    return rgba_arr.view('u4')

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
    Create RGB point cloud from ordinary nx4 (r,g,b,a) array, or equivalent list representation.
    Note that RGB value should be represented in integer of 0~255
    '''
    data = _check_point_size(data, 4)
    return PointCloud(_parse_rgba(data, 0).view(dict(names=['rgba'], formats=['u4'])), 'RGB')

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

    dt = np.dtype(dict(names=['x','y','z','rgba'], formats=['f4','f4','f4','u4'], offsets=[0,4,8,16], itemsize=32))
    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['rgba'] = data[:,0], data[:,1], data[:,2], _parse_rgba(data, 3)
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
    arr['curvature'] = data[:,3] if data.shape[-1] == 4 else 0
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

def create_xyzn(data):
    data = np.asarray(data)
    if data.shape[-1] not in [6, 7]:
        raise ValueError("Each point should contain 6 or 7 values")

    dt = np.dtype(dict(
        names=['x','y','z','normal_x', 'normal_y', 'normal_z', 'curvature'],
        formats=['f4','f4','f4','f4','f4','f4','f4'],
        offsets=[0,4,8,16,20,24,32], itemsize=48))

    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'] = data[:,0], data[:,1], data[:,2]
    arr['normal_x'], arr['normal_y'], arr['normal_z'] = data[:,3], data[:,4], data[:,5]
    arr['curvature'] = data[:,6] if data.shape[-1] == 7 else 0
    return PointCloud(arr, 'XYZN')

def create_xyzrgbn(data):
    data = np.asarray(data)
    if data.shape[-1] not in [9, 10]:
        raise ValueError("Each point should contain 9 or 10 values")

    dt = np.dtype(dict(
        names=['x','y','z','normal_x', 'normal_y', 'normal_z', 'rgb', 'curvature'],
        formats=['f4','f4','f4','f4','f4','f4','u4','f4'],
        offsets=[0,4,8,16,20,24,32,36], itemsize=48))

    arr = np.empty(len(data), dtype=dt)
    arr['x'], arr['y'], arr['z'] = data[:,0], data[:,1], data[:,2]
    arr['normal_x'], arr['normal_y'], arr['normal_z'] = data[:,3], data[:,4], data[:,5]
    arr['rgb'], arr['curvature'] = _parse_rgb(data, 6), (data[:,9] if data.shape[-1] == 10 else 0)
    return PointCloud(arr, 'XYZRGBN')
