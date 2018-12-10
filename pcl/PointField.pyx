from pcl._PCLPointField cimport PCLPointField

cdef dict _type_mapping = {
    1: 'i1',
    2: 'u1',
    3: 'i2',
    4: 'u2',
    5: 'i4',
    6: 'u4',
    7: 'f4',
    8: 'f8'
}

cdef class PointField:
    property name:
        def __get__(self): return self.base.name.decode('ascii')
    
    property count:
        def __get__(self): return self.base.count

    property offset:
        def __get__(self): return self.base.offset

    property datatype:
        def __get__(self): return _type_mapping[self.base.datatype]

    def __repr__(self):
        return "<PointField {0} : {1}>".format(self.name, self.datatype)

    @staticmethod
    cdef PointField wrap(PCLPointField& data):
        cdef PointField obj = PointField.__new__(PointField)
        obj.base = data
        return obj
