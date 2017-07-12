'''
Implementation of following files:
    eigen3/Eigen/src/Geometry/Quaternion.h

Interface for quaterion.
A wrapper for numpy-quaternion, for numpy-quaternion is under developing
'''
import numpy as np
import quaternion

#TODO: decide whether a quaternion class is essential

class Quaternion:
    '''
    Implement basic operations with quaternion. Quaternion is mostly used in rotation computing
    '''

    def __init__(self, init=None):
        if isinstance(init, type(self)):
            self.__quat = init.__quat
        elif isinstance(init, np.quaternion):
            self.__quat = init
        else:
            self.__quat = quaternion.as_quat_array(np.array(init, copy=False))

    @staticmethod
    def identity():
        '''
        Get a identity quaternion
        '''
        return Quaternion(np.quaternion(1, 0, 0, 0))

    def tolist(self):
        '''
        Convert quaternion to float list
        '''
        return quaternion.as_float_array(self.__quat).tolist()

    def __str__(self):
        return str(self.__quat)

    def __eq__(self, target):
        return self.__quat == target.__quat

    def __reduce__(self):
        return type(self), (quaternion.as_float_array(self.__quat),)
