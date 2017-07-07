'''
Tests of pcl.io.file_io
'''

import os
import sys
from io import StringIO, BytesIO
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.io as pio

TEST_ROOT = os.path.dirname(__file__) + '/' # 'test/'

def test_pcd_reader():
    '''
    Test PCDReader
    '''
    reader = pio.PCDReader()
    meta = reader.read_header(TEST_ROOT + 'data/cturtle.pcd')
    assert meta['data_type'] == 'binary_compressed'
    assert reader.check_header(meta)

    # ascii
    pcd, _ = reader.read(TEST_ROOT + 'data/curve2d.pcd')
    assert len(pcd) == 51
    assert pcd.names == ['x', 'y', 'z']
    # binary
    pcd, _ = reader.read(TEST_ROOT + 'data/curve2d_binary.pcd')
    assert len(pcd) == 110
    assert pcd.names == ['x', 'y', 'z']
    # binary_compressed
    pcd, _ = reader.read(TEST_ROOT + 'data/brisk_image.pcd')
    assert len(pcd) == 327680
    assert pcd.names == ['rgba']

def test_pcd_writer():
    '''
    Test PCDWriter
    '''
    reader = pio.PCDReader()
    writer = pio.PCDWriter()
    cloud = pcl.PointCloud([(1, 2, 3), (4, 5, 6)], ['x', 'y', 'z'])
    cloud.append(cloud) # too few data will cause compression errors
    headerstr = writer.generate_header(cloud, 'ascii')
    header = reader.read_header(StringIO(headerstr))
    assert header['type'] == ['I', 'I', 'I']
    assert header['fields'] == ['x', 'y', 'z']
    assert header['count'] == [1, 1, 1]

    #ascii
    buf = StringIO()
    writer.write_ascii(buf, cloud)
    compare, _ = reader.read(buf)
    buf.close()
    assert (compare.data == cloud.data).all()
    #binary
    buf = BytesIO()
    writer.write_binary(buf, cloud)
    compare, _ = reader.read(buf)
    buf.close()
    assert (compare.data == cloud.data).all()
    #binary_compressed
    buf = BytesIO()
    writer.write_binary_compressed(buf, cloud)
    compare, _ = reader.read(buf)
    buf.close()
    assert (compare.data == cloud.data).all()

    #global function test
    buf = BytesIO()
    pio.savepcd(buf, cloud)
    compare = pio.loadpcd(buf)
    buf.close()
    assert (compare.data == cloud.data).all()

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
# test_pcd_writer()
