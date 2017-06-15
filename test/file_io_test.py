'''
Tests of pcl.io.file_io
'''

import os
import sys
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.io as pio

def test_pcd_reader():
    '''
    Test PCDReader
    '''
    reader = pio.PCDReader()
    meta = reader.read_header('test/data/cturtle.pcd')
    assert meta['data_type'] == 'binary_compressed'
    assert reader.check_header(meta)

    # ascii
    pcd, _ = reader.read('test/data/curve2d.pcd')
    assert len(pcd) == 51
    assert pcd.names == ['x', 'y', 'z']
    # binary
    pcd, _ = reader.read('test/data/curve2d_binary.pcd')
    assert len(pcd) == 110
    assert pcd.names == ['x', 'y', 'z']
    # binary_compressed
    pcd, _ = reader.read('test/data/brisk_image.pcd')
    assert len(pcd) == 327680
    assert pcd.names == ['rgba']

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
# test_pcd_reader()
