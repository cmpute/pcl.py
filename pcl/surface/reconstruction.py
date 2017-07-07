'''
Implementation of following files:
    pcl/surface/include/pcl/surface/reconstruction.h
'''

import abc
from ..common import _CloudBase

class _SurfaceBase(_CloudBase, metaclass=abc.ABCMeta):
    '''
    All types of meshing/reconstruction
    algorithms in pcl.surface must inherit from this, in order to make
    sure we have a consistent API.
    '''
    def __init__(self, cloud=None, indices=None, search=None):
        super().__init__(cloud, indices)
        self.search_method = search

    @abc.abstractmethod
    def reconstruct(self):
        '''
        Base method for surface reconstruction for all points given in input cloud

        # Returns
        output : PolygonMesh
            The resultant reconstructed surface model
        '''
        pass

class SurfaceReconstruction(_SurfaceBase, metaclass=abc.ABCMeta):
    '''
    SurfaceReconstruction represents a base surface reconstruction
    class. All surface reconstruction methods take in a point cloud and
    generate a new surface from it, by either re-sampling the data or
    generating new data altogether. These methods are thus not preserving
    the topology of the original data.

    Reconstruction methods that always preserve the original input
    point cloud data as the surface vertices and simply construct the mesh on
    top should inherit from MeshConstruction.
    '''
    def __init__(self, cloud=None, indices=None, search=None):
        super().__init__(cloud, indices, search)
        self._check_tree = True

    def reconstruct(self):
        '''
        Base method for surface reconstruction for all points given in input cloud

        # Returns
        output : PolygonMesh
            The resultant reconstructed surface model
        '''
        pass # TODO: Not Implemented

    def reconstruct_cloud(self):
        '''
        Base method for surface reconstruction for all points given in input cloud

        # Returns
        points : PointCloud
            The resultant points lying on the new surface
        polygons : list of vertices
            The resultant polygons, as a set of vertices. The Vertices structure contains an
            array of point indices.
        '''
        pass # TODO: Not Implemented

    @abc.abstractmethod
    def _perform_reconstruction(self):
        '''
        Abstract surface reconstruction method.

        # Returns
        output : PolygonMesh
            The output polygonal mesh
        '''
        pass

    @abc.abstractmethod
    def _perform_cloud_reconstruction(self):
        '''
        Abstract surface reconstruction method.

        # Returns
        points : PointCloud
            The resultant points lying on the new surface
        polygons : list of vertices
            The resultant polygons, as a set of vertices. The Vertices structure contains an
            array of point indices.
        '''
        pass

class MeshConstruction(_SurfaceBase):
    '''
    MeshConstruction represents a base surface reconstruction
    class. All mesh constructing methods that take in a point cloud and
    generate a surface that uses the original data as vertices should inherit
    from this class.

    Reconstruction methods that generate a new surface or create new
    vertices in locations different than the input data should inherit from
    SurfaceReconstruction.
    '''
    def __init__(self, cloud=None, indices=None, search=None):
        super().__init__(cloud, indices, search)
        self._check_tree = True

    def reconstruct(self):
        '''
        Base method for surface reconstruction for all points given in input cloud

        # Returns
        output : PolygonMesh
            The resultant reconstructed surface model
        '''
        pass # TODO: Not Implemented

    def reconstruct_cloud(self):
        '''
        Base method for surface reconstruction for all points given in input cloud

        # Returns
        polygons : list of vertices
            The resultant polygons, as a set of vertices. The Vertices structure contains an
            array of point indices.
        '''
        pass # TODO: Not Implemented

    @abc.abstractmethod
    def _perform_reconstruction(self):
        '''
        Abstract surface reconstruction method.

        # Returns
        output : PolygonMesh
            The output polygonal mesh
        '''
        pass

    @abc.abstractmethod
    def _perform_cloud_reconstruction(self):
        '''
        Abstract surface reconstruction method.

        # Returns
        polygons : list of vertices
            The resultant polygons, as a set of vertices. The Vertices structure contains an
            array of point indices.
        '''
        pass
