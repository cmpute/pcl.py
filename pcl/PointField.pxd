from pcl.common.PCLPointField cimport PCLPointField
import numpy as np
cimport numpy as cnp

cdef dict _FIELD_TYPE_MAPPING
cdef inline int _field_type_map_inv(cnp.dtype dt):
    if dt.type == np.int8:
        return 1
    elif dt.type == np.uint8:
        return 2
    elif dt.type == np.int16:
        return 3
    elif dt.type == np.uint16:
        return 4
    elif dt.type == np.int32:
        return 5
    elif dt.type == np.uint32:
        return 6
    elif dt.type == np.float32:
        return 7
    elif dt.type == np.float64:
        return 8

cdef class PointField:
    cdef PCLPointField base
    cdef object __weakref__

    @staticmethod
    cdef PointField wrap(const PCLPointField& data)
