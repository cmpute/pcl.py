from libc.stdint cimport uint64_t, uint8_t
from pcl.common.PCLHeader cimport PCLHeader
from pcl.common.PCLPointField cimport PCLPointField
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2

cdef inline object to_msg_header(const PCLHeader &src) except +:
    from std_msgs.msg import Header
    from rospy import Time
    
    cdef object dst = Header()
    dst.seq = src.seq
    dst.frame_id = src.frame_id
    dst.stamp = Time(nsecs=src.stamp*1000)
    return dst
cdef inline void from_msg_header(object src, PCLHeader &dst) except +:
    dst.seq = src.seq
    dst.frame_id = src.frame_id
    dst.stamp = src.stamp.nsecs // 1000

cdef inline object to_msg_field(const PCLPointField &src) except +:
    from sensor_msgs.msg import PointField
    cdef object dst = PointField()
    dst.name = src.name
    dst.offset = src.offset
    dst.datatype = src.datatype
    dst.count = src.count
    return dst
cdef inline void from_msg_field(object src, PCLPointField &dst) except +:
    dst.name = src.name
    dst.offset = src.offset
    dst.datatype = src.datatype
    dst.count = src.count

# not using const to prevent const iterator issue (https://github.com/cython/cython/issues/1451)
cdef inline object to_msg_cloud(PCLPointCloud2 &src) except +:
    from sensor_msgs.msg import PointCloud2
    cdef object dst = PointCloud2()
    dst.header = to_msg_header(src.header)
    dst.height = src.height
    dst.width = src.width
    for field in src.fields:
        temp_field = to_msg_field(field)
        dst.fields.append(temp_field)
    dst.is_bigendian = src.is_bigendian
    dst.point_step = src.point_step
    dst.row_step = src.row_step
    dst.data = src.data
    dst.is_dense = src.is_dense
    return dst
cdef inline void from_msg_cloud(object src, PCLPointCloud2 &dst) except +:
    cdef PCLPointField temp_field
    cdef uint8_t* dptr
    from_msg_header(src.header, dst.header)
    dst.height = src.height
    dst.width = src.width
    for field in src.fields:
        temp_field = PCLPointField()
        from_msg_field(field, temp_field)
        dst.fields.push_back(temp_field)
    dst.is_bigendian = src.is_bigendian
    dst.point_step = src.point_step
    dst.row_step = src.row_step
    dptr = src.data
    dst.data.assign(dptr, dptr + len(src.data))
    dst.is_dense = src.is_dense
