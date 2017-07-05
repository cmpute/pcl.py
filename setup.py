''' Setup File '''
import os
from setuptools import setup, find_packages

_VER_MAJOR = 0
_VER_MINOR = 0
_VER_MICRO = 3  # use '' for first of series, number for 1 and above
_VER_EXTRA = None  # Uncomment this for full releases

# Construct full version string from these.
_VER = [_VER_MAJOR, _VER_MINOR]
if _VER_MICRO:
    _VER.append(_VER_MICRO)
if _VER_EXTRA:
    _VER.append(_VER_EXTRA)

__version__ = '.'.join(map(str, _VER))

def _read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(name='PyPCL',
      version=__version__,
      description='Python implementation of Point Cloud Library (PCL)',
      long_description=_read('README.md'),
      author='Jacob Zhong',
      author_email='cmpute@gmail.com',
      url='https://github.com/cmpute/pypcl',
      download_url='',
      license='BSD-3-Clause',
      install_requires=['numpy', 'numpy-quaternion'],
      extras_require={
          'compress': ['lzf'],
          'search': ['nmslib'],
          'test': ['pytest'],
      },
      packages=find_packages()) # 'pcl'
