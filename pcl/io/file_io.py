'''
Implementation of following files:
    pcl/io/include/pcl/io/file_io.h
    pcl/io/include/pcl/io/pcd_io.h
    pcl/io/include/pcl/io/impl/pcd_io.hpp
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

def _check_file(file, offset=0, ext=None, fileopt='r'):
    # if file is not a file-like object, then open it with file as a path string
    # offset is applied then

    if isinstance(file, str):
        if ext is not None and not file.endswith('.' + ext):
            file += '.' + ext
        fid = open(file, fileopt)
        own = True
    else:
        fid = file
        own = False
    fid.seek(offset, 0)
    return fid, own

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
    def write(self, file, cloud, **opts):
        '''Save point cloud data containing n-D points to file

        # Parameters
            file : file-like object or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
            opts : dict
                additional options for the writer
        '''
        pass

class PCDReader(_FileReader):
    '''
    Point Cloud Data (PCD) file format reader.
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
        file, own = _check_file(file, offset, 'pcd', 'rb')

        try:
            hlines = []
            line = ''
            while not line.startswith('DATA'):
                line = file.readline().strip()
                if isinstance(line, bytes):
                    line = line.decode('ascii')
                if len(line) > 0:
                    hlines.append(line)
            data_offset = file.tell()
        finally:
            if own:
                file.close()

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
        file, own = _check_file(file, offset, 'pcd', 'rb')

        try:
            header = []
            line = ''
            while not line.startswith('DATA'):
                line = file.readline().strip()
                if isinstance(line, bytes):
                    line = line.decode('ascii')
                if len(line) > 0:
                    header.append(line)
            header = self.__parse_header(header)
            self.check_header(header) # throw if important fields are not invalid

            fields = [(_f, _s, _t, _c) for _f, _s, _t, _c in zip(
                header['fields'], header['size'], header['type'], header['count'])]
            fields, _ = _cast_fields_to_tuples(fields)
            dtype = np.dtype(fields)

            if header['data_type'] == 'ascii':
                data = np.loadtxt(file, dtype=dtype, delimiter=' ')
            elif header['data_type'] == 'binary':
                buf = file.read(header['points']*dtype.itemsize)
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
                    fmt, file.read(struct.calcsize(fmt)))
                compressed_data = file.read(compressed_size)

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
        finally:
            if own:
                file.close()

        params = {'points': data, 'fields': fields, 'copy': False}
        for field in ('width', 'height'):
            if field in header:
                params[field] = header[field]
        cloud = PointCloud(**params)

        if 'viewpoint' in header:
            cloud.sensor_orientation = Quaternion(header['viewpoint'][3:])
            cloud.sensor_origin = np.array(header['viewpoint'][:3])

        ver = header['version'] if 'version' in header else .7
        return cloud, ver

class PCDWriter(_FileWriter):
    '''
    Point Cloud Data (PCD) file format writer.
    '''

    def __parse_descr(self, descr):
        # parse description string into (size, type, count)
        digipre = list()
        digisuf = list()
        pre = True
        for chara in descr:
            if str.isdigit(chara):
                if pre:
                    digipre.append(chara)
                else:
                    digisuf.append(chara)
            else:
                dtype = chara.upper()
                pre = False
        if len(digipre) == 0:
            return ''.join(digisuf), dtype, '1'
        else:
            return ''.join(digisuf), dtype, ''.join(digipre)

    def generate_header(self, cloud, data_type):
        """
        Generate the header of a PCD file format

        # Returns
            header : str
                PCD format header string
        """
        template = """\
VERSION {version}
FIELDS {fields}
SIZE {size}
TYPE {type}
COUNT {count}
WIDTH {width}
HEIGHT {height}
VIEWPOINT {viewpoint}
POINTS {points}
DATA {data}
"""
        fields = list()
        sizes = list()
        types = list()
        counts = list()
        for name, descr in cloud.fields:
            fields.append(name)
            size, dtype, count = self.__parse_descr(descr)
            sizes.append(size)
            types.append(dtype)
            counts.append(count)

        header = {'version': .7}
        header['fields'] = ' '.join(fields)
        header['type'] = ' '.join(types)
        header['size'] = ' '.join(sizes)
        header['count'] = ' '.join(counts)
        header['width'] = cloud.width
        header['height'] = cloud.height
        header['viewpoint'] = ' '.join(map(str, cloud.sensor_origin[:3].tolist())) + ' '
        header['viewpoint'] += ' '.join(map(str, cloud.sensor_orientation.tolist()))
        header['points'] = len(cloud)
        header['data'] = data_type
        return template.format(**header)

    def write_ascii(self, file, cloud):
        '''
        Save point cloud data to a PCD file containing n-D points, in ASCII format

        # Parameters
            file : file-like object or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
        '''
        fields = [] # format string for fields
        for _, descr in cloud.fields:
            _, dtype, count = self.__parse_descr(descr)
            if dtype == 'F':
                fmtstr = '%.10f'
            elif dtype == 'I':
                fmtstr = '%d'
            elif dtype == 'U':
                fmtstr = '%u'
            count = int(count)
            if count > 1:
                fmtstr = ' '.join([fmtstr] * count)
            fields.append(fmtstr)

        file, own = _check_file(file, 0, 'pcd', 'w')

        try:
            file.write(self.generate_header(cloud, 'ascii'))
            for point in cloud:
                line = []
                for idx, data in enumerate(tuple(point)):
                    line.append(fields[idx] % data)
                file.write(' '.join(line))
                file.write('\n')
        finally:
            if own:
                file.close()

    def write_binary(self, file, cloud):
        '''
        Save point cloud data to a PCD file containing n-D points, in binary format

        # Parameters
            file : file-like object or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
        '''
        file, own = _check_file(file, 0, 'pcd', 'wb')

        try:
            file.write(self.generate_header(cloud, 'binary').encode('ascii'))
            file.write(cloud.data.tostring('C'))
        finally:
            if own:
                file.close()

    def write_binary_compressed(self, file, cloud):
        '''
        Save point cloud data to a PCD file containing n-D points, in compressed binary format

        # Parameters
            file : file-like object or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
        '''
        import struct
        try:
            import lzf
        except ImportError:
            raise ImportError(
                'lzf decompression lib is required to read compressed .pcd file.' +
                'lzf can be install from setup.py in https://github.com/teepark/python-lzf')

        file, own = _check_file(file, 0, 'pcd', 'wb')

        try:
            file.write(self.generate_header(cloud, 'binary_compressed').encode('ascii'))

            uncompressed_lst = []
            for field in cloud.names:
                column = np.ascontiguousarray(cloud.data[field]).tostring('C')
                uncompressed_lst.append(column)
            uncompressed = b''.join(uncompressed_lst)
            uncompressed_size = len(uncompressed)
            buf = lzf.compress(uncompressed)
            if buf is None:
                warnings.warn("compression didn't shrink the file during processing, \
                               which may cause data loss")
                buf = uncompressed
                compressed_size = uncompressed_size
            else:
                compressed_size = len(buf)
            fmt = 'II'
            file.write(struct.pack(fmt, compressed_size, uncompressed_size))
            file.write(buf)
        finally:
            if own:
                file.close()

    def write(self, file, cloud, **opts):
        '''Save point cloud data to a PCD file containing n-D points

        # Parameters
            file : file-like object or str
                The output file or name of it.
            cloud : PointCloud
                The the point cloud data that need saving
            opts : dict
                additional options for the writer
        '''
        if 'binary' in opts and isinstance(opts['binary'], bool):
            binary = opts['binary']
        else:
            binary = True # binary by default

        if binary:
            if 'compress' in opts and isinstance(opts['compress'], bool):
                compress = opts['compress']
            else:
                compress = False # not compressed by default

        if binary:
            if compress:
                self.write_binary_compressed(file, cloud)
            else:
                self.write_binary(file, cloud)
        else:
            self.write_ascii(file, cloud)

def loadpcd(file):
    '''
    Load point cloud from ''.pcd'' file

    # Parameter
        file : file-like object or str
            The file or the name of it to load.
            File-like object need to support seek() and read()
    '''
    cloud, _ = PCDReader().read(file)
    return cloud

def savepcd(file, cloud, binary=True, compress=False):
    '''
    Save point cloud into ''.pcd'' file

    # Parameter
        file : file-like object or str
            The file or the name of it to load.
            File-like object need to support seek() and read()
        cloud : PointCloud
            The the point cloud data that need saving
        binary : bool
            Save the file with binary format if true,
                otherwise data is saved in ascii format.
        compress : bool
            Indicating whether compress binary data when saving
    '''
    PCDWriter().write(file, cloud, binary=binary, compress=compress)
