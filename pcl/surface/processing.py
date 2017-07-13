'''
Implementation of following files:
    pcl/surface/include/pcl/surface/impl/processing.hpp
    pcl/surface/include/pcl/surface/processing.h
    pcl/surface/src/processing.cpp  
'''

import abc
from ..common import _CloudBase

class CloudSurfaceProcessing(_CloudBase, metaclass=abc.ABCMeta):
    '''
    CloudSurfaceProcessing represents the base class for algorithms that takes a point cloud as
    input and produces a new output cloud that has been modified towards a better surface
    representation. These types of algorithms include surface smoothing, hole filling, cloud
    upsampling etc.
    '''
    def __init__(self, cloud=None, indices=None):
        super().__init__(cloud, indices)

    def process(self):
        '''
        Process the input cloud and store the results

        # Returns
        output : PointCloud
            The cloud where the results will be stored
        '''
        pass # TODO: Not Implemented

    @abc.abstractmethod
    def _perform_processing(self):
        '''
        Abstract cloud processing method
        '''
        pass

class MeshProcessing(metaclass=abc.ABCMeta):
    '''
    MeshProcessing represents the base class for mesh processing algorithms.
    '''
    def __init__(self, mesh=None):
        self._input_mesh = mesh

    @property
    def input_mesh(self):
        '''
        Get the input mesh to be processed
        '''
        return self._input_mesh

    @input_mesh.setter
    def input_mesh(self, value):
        '''
        Set the input mesh that we want to process
        '''
        self._input_mesh = value

    def process(self):
        '''
        Process the input surface mesh and store the results

        # Returns
        output : PolygonMesh
            The resultant processed surface model
        '''
        pass # TODO: Not Implemented

    @abc.abstractmethod
    def _perform_processing(self):
        '''
        Abstract surface processing method.
        '''
        pass

    def _init_compute(self):
        '''
        Initialize computation. Must be called before processing starts.
        '''
        if self.input_mesh is None:
            return False
        return True

    def _deinit_compute(self):
        '''
        UnInitialize computation. Must be called after processing ends
        '''
        pass
