'''
Implementation of following files:
    pcl/io/include/pcl/io/file_io.h
    pcl/io/src/file_io.h
'''
from __future__ import absolute_import

import abc
import numpy as np

class _FileReader(metaclass=abc.ABCMeta):
    '''
    Point Cloud Data (FILE) file format reader interface.
    Any (FILE) format file reader should implement its virtual methodes.
    '''

    @abc.abstractmethod
    def read_header(self, file_name, cloud, offset=0):
        '''Read a point cloud data header from a FILE file.

        Load only the meta information (number of points, their types, etc),
        and not the points themselves, from a given FILE file. Useful for fast
        evaluation of the underlying data structure.

        # Parameters
            file_name : str
                The name of the file to load
            cloud : PointCloud
                The resultant point cloud dataset (only the header will be filled)
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            origin : Vector
                The sensor acquisition origin (only for > FILE_V7 - null if not present)
            orientation : Quaternion
                The sensor acquisition orientation (only for > FILE_V7 - identity if not present)
            file_version : int
                The FILE version of the file (either FILE_V6 or FILE_V7)
            data_type : int
                The type of data (binary data=1, ascii=0, etc)
            data_idx : int
                The offset of cloud data within the file
        '''
        pass

    @abc.abstractmethod
    def read(self, file_name, cloud, offset=0):
        '''Read a point cloud data from a FILE file and store it into a pcl/PCLPointCloud2.

        # Parameters
            file_name : str
                The name of the file containing the actual PointCloud data
            cloud : PointCloud
                The resultant PointCloud message read from disk
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            origin : Vector
                The sensor acquisition origin (only for > FILE_V7 - null if not present)
            orientation : Vector
                The sensor acquisition orientation (only for > FILE_V7 - identity if not present)
            file_version : int
                The FILE version of the file (either FILE_V6 or FILE_V7)
        '''
        pass

class _FileWriter(metaclass=abc.ABCMeta):
    '''
    Point Cloud Data (FILE) file format writer.
    Any (FILE) format file reader should implement its virtual methodes
    '''

    @abc.abstractmethod
    def write(self, file_name, cloud, binary=False):
        '''Save point cloud data to a FILE file containing n-D points

        Load only the meta information (number of points, their types, etc),
        and not the points themselves, from a given FILE file. Useful for fast
        evaluation of the underlying data structure.

        # Parameters
            file_name : str
                The output file name
            cloud : PointCloud
                The resultant point cloud dataset (only the header will be filled)

        # Returns
            file_version : int
                The FILE version of the file (either FILE_V6 or FILE_V7)
            data_type : int
                The type of data (binary data=1, ascii=0, etc)
            data_idx : int
                The offset of cloud data within the file
        '''
        pass
    
    # TODO: Unfinished File 