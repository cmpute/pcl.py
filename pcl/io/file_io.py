'''
Implementation of following files:
    pcl/io/include/pcl/io/file_io.h
    pcl/io/src/file_io.h
    pcl/io/src/pcd_io.cpp
'''
from __future__ import absolute_import

import abc
import re
import warnings
import numpy as np
from ..pointcloud import _cast_fields_to_tuples, PointCloud
from ..quaternion import Quaternion

def _check_rfile(file, offset=0, ext=None):
    # if file is not a file-like object, then open it with file as a path string
    # offset is applied then

    if isinstance(file, str):
        if ext is not None and not file.endswith('.' + ext):
            file += '.' + ext
        fid = open(file, "rb")
    else:
        fid = file
    fid.seek(offset, 0)
    return fid

class _FileReader(metaclass=abc.ABCMeta):
    '''
    Point Cloud Data (FILE) file format reader interface.
    Any (FILE) format file reader should implement its virtual methodes.
    '''

    @abc.abstractmethod
    def read_header(self, file, offset=0):
        '''Read a point cloud data header from file.

        Load only the meta information (number of points, their types, etc),
        and not the points themselves, from a given FILE file. Useful for fast
        evaluation of the underlying data structure.

        # Parameters
            file : file-like object or str
                The file or the name of it to load.
                File-like object need to support seek() and read()
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            A dictionary of the headers. Headers are not reinterpreted.
        '''
        pass

    @abc.abstractmethod
    def read(self, file, offset=0):
        '''Read a point cloud data from file and store it into a pcl.PointCloud.

        # Parameters
            file : file-like object or str
                The file containing the actual PointCloud data or the name of it
                File-like object need to support seek() and read()
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            cloud : PointCloud
                The resultant PointCloud data read from disk
            extra : misc (tuple)
                Additional information

        Thus the function should be used as 'pc, _=reader.read(...)'
        '''
        pass

class _FileWriter(metaclass=abc.ABCMeta):
    '''
    Point Cloud Data (FILE) file format writer.
    Any (FILE) format file reader should implement its virtual methodes
    '''

    @abc.abstractmethod
    def write(self, file, cloud, binary=False):
        '''Save point cloud data to a FILE file containing n-D points

        Load only the meta information (number of points, their types, etc),
        and not the points themselves, from a given FILE file. Useful for fast
        evaluation of the underlying data structure.

        # Parameters
            file : file or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
            binary : bool
                Set to true if the file is to be written in a binary
        '''
        pass

class PCDReader(_FileReader):
    '''
    Support reading from .pcd files
    '''

    def __parse_header(self, lines):
        header = {}
        for line in lines:
            if line.startswith('#') or len(line) < 2:
                continue
            match = re.match(r'(\w+)\s+([\w\s\.]+)', line)
            if not match:
                warnings.warn("warning: can't understand line: %s" % line)
                continue
            key, value = match.group(1).lower(), match.group(2)
            if key == 'columns':
                key = 'fields' # for compability
            if key == 'version':
                header[key] = float(value)
            elif key in ('fields', 'type'):
                header[key] = value.split()
            elif key in ('size', 'count'):
                header[key] = list(map(int, value.split()))
            elif key in ('width', 'height', 'points'):
                header[key] = int(value)
            elif key == 'viewpoint':
                header[key] = list(map(float, value.split()))
            elif key == 'data':
                header['data_type'] = value.strip().lower()
        return header

    def check_header(self, header):
        '''
        Check whether the header is valid.

        If important constraints are not satisfied, TypeError will be raised,
        otherwise some warnings are sent out
        # Returns
            valid : bool
                True if no warning is sent out and no error is raised
        '''
        valid = True

        required = ('version', 'fields', 'size', 'width', 'height', 'points',
                    'viewpoint', 'data_type', 'type')
        essential = ('fields', 'size', 'type', 'count', 'data_type')
        for key in required:
            if not key in header:
                message = 'missing %s field' % key.upper()
                if key in essential:
                    raise TypeError(message)
                else:
                    warnings.warn(message)
                valid = False

        if not len(header['fields']) == len(header['type']) == \
               len(header['size']) == len(header['count']):
            raise TypeError('length of FIELD, TYPE, SIZE and COUNT must be equal')

        if not header['data_type'] in ('ascii', 'binary', 'binary_compressed'):
            raise TypeError('unknown data type: %s ,' + header['data_type'] +
                            'should be ascii/binary/binary_compressed')
        elif header['data_type'] is not 'ascii' and 'points' not in header:
            raise TypeError('POINTS field is essential for reading binary pcd file')

        if 'height' in header and 'width' in header and 'points' in header:

            for key in ('height', 'width', 'points'):
                if not header[key] > 0:
                    warnings.warn('%s must be positive number' % key.upper() +
                                  'field value will be replaced by default value')
                    valid = False

            if header['width'] * header['height'] != header['points']:
                raise TypeError('WIDTH x HEIGHT is not equal with POINTS')

        if 'viewpoint' in header:
            if not len(header['viewpoint']) == 7:
                raise TypeError('Not enough number of elements in VIEWPOINT!' +
                                'Need 7 values (tx ty tz qw qx qy qz).')

        return valid

    def read_header(self, file, offset=0):
        '''
        Read a point cloud data header from .pcd file.

        # Parameters
            file : file-like object or str
                The file or the name of it to load.
                File-like object need to support seek() and read()
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            A dictionary of the headers. Headers are not reinterpreted
                (except that DATA is renamed as data_type).
            Extra keys (in addition to what a point cloud needs) of the dictionary are listed below
            version : float
                The FILE version of the file (either .6(FILE_V6) or .7(FILE_V7))
            data_type : str
                The type of data (ascii, binary, binary_compressed)
            data_offset : int
                The offset of raw cloud data within the file
        '''
        with _check_rfile(file, offset, 'pcd') as fileo:
            hlines = []
            line = ''
            while not line.startswith('DATA'):
                line = fileo.readline().strip().decode('ascii')
                if len(line) > 0:
                    hlines.append(line)
            data_offset = fileo.tell()
        header = self.__parse_header(hlines)
        header['data_offset'] = data_offset
        return header

    def read(self, file, offset=0):
        '''Read a point cloud data from .pcd file and store it into a pcl.PointCloud.

        # Parameters
            file : file-like object or str
                The file containing the actual PointCloud data or the name of it
                File-like object need to support seek() and read()
            offset : int
                The offset in the file where to expect the true header to begin.

        # Returns
            cloud : PointCloud
                The resultant PointCloud data read from disk
            version : float
                The FILE version of the file (either .6(FILE_V6) or .7(FILE_V7))
        '''
        with _check_rfile(file, offset, 'pcd') as fileo:
            header = []
            line = ''
            while not line.startswith('DATA'):
                line = fileo.readline().strip().decode('ascii')
                if len(line) > 0:
                    header.append(line)
            header = self.__parse_header(header)
            self.check_header(header) # throw if important fields are not invalid

            fields = [(_f, _s, _t, _c) for _f, _s, _t, _c in zip(
                header['fields'], header['size'], header['type'], header['count'])]
            fields, _ = _cast_fields_to_tuples(fields)
            dtype = np.dtype(fields)

            if header['data_type'] == 'ascii':
                data = np.loadtxt(fileo, dtype=dtype, delimiter=' ')
            elif header['data_type'] == 'binary':
                buf = fileo.read(header['points']*dtype.itemsize)
                data = np.fromstring(buf, dtype=dtype)
            elif header['data_type'] == 'binary_compressed':
                # the conversion method is from 'pypcd' lib
                import struct
                try:
                    import lzf
                except ImportError:
                    raise ImportError(
                        'lzf decompression lib is required to read compressed .pcd file.' +
                        'lzf can be install from setup.py in https://github.com/teepark/python-lzf')

                fmt = 'II'
                compressed_size, uncompressed_size = struct.unpack(
                    fmt, fileo.read(struct.calcsize(fmt)))
                compressed_data = fileo.read(compressed_size)

                buf = lzf.decompress(compressed_data, uncompressed_size)
                if len(buf) != uncompressed_size:
                    raise Exception('Error decompressing data')
                # the data is stored field-by-field
                data = np.zeros(header['points'], dtype=dtype)
                ptr = 0
                for index, name in enumerate(dtype.names):
                    subtype = dtype[index]
                    step = subtype.itemsize * header['points']
                    column = np.fromstring(buf[ptr:(ptr + step)], subtype)
                    data[name] = column
                    ptr += step
            # No else, for that the type is checked in header checking

        params = {'points': data, 'fields': fields, 'copy': False}
        for field in ('width', 'height'):
            if field in header:
                params[field] = header[field]
        cloud = PointCloud(**params)

        if 'viewpoint' in header:
            # TODO: Need check the conversion
            cloud.sensor_orientation = Quaternion(header['viewpoint'][3:])
            cloud.sensor_origin = np.array(header['viewpoint'][:3])

        ver = header['version'] if 'version' in header else .7
        return cloud, ver
