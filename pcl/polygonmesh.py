'''
Implementation of following files:
    pcl/common/include/pcl/PolygonMesh.h
    pcl/common/include/pcl/Vertices.h

TODO: This file will be changing. Vertices class may not be necessary.
'''

class Vertices:
    '''
    Implement of pcl::Vertices
    '''
    def __init__(self, vertices):
        self.vertices = vertices

class PolygonMesh:
    '''
    Implement of pcl::PolygonMesh

    TODO: Judge differences between pcl::PolygonMesh and pcl::geometry::PolygonMesh
    '''
    def __init__(self, cloud, polygon):
        self.cloud = cloud
        self.polygon = polygon
