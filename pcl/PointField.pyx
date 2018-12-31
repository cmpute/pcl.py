from pcl.ros import ros_exist, ros_error
if ros_exist: from .ros cimport from_msg_field, to_msg_field
from pcl.common.PCLPointField cimport PCLPointField

cdef dict _FIELD_TYPE_MAPPING = {
    1: ('i1', 1),
    2: ('u1', 1),
    3: ('i2', 2),
    4: ('u2', 2),
    5: ('i4', 4),
    6: ('u4', 4),
    7: ('f4', 4),
    8: ('f8', 8),
}

cdef class PointField:
    def __init__(self, data=None):
        if ros_exist:
            from sensor_msgs.msg import PointField as rospf
            if isinstance(data, rospf):
                from_msg_field(data, self.base)

    property name:
        def __get__(self): return self.base.name.decode('ascii')
    
    property count:
        def __get__(self): return self.base.count

    property offset:
        def __get__(self): return self.base.offset

    property datatype:
        def __get__(self): return _FIELD_TYPE_MAPPING[self.base.datatype][0]

    def __repr__(self):
        return "<PointField {0} : {1}>".format(self.name, self.datatype)

    def to_msg(self):
        if not ros_exist: raise ros_error
        return to_msg_field(self.base)

    @staticmethod
    cdef PointField wrap(PCLPointField& data):
        cdef PointField obj = PointField.__new__(PointField)
        obj.base = data
        return obj
