''' Setup File '''

try:
    from setuptools import setup, find_packages
except ImportError:
    # from distutils.core import setup
    raise ImportError('setuptools is required for installing')

_VER_MAJOR = 0
_VER_MINOR = 0
_VER_MICRO = 5
_VER_EXTRA = 'a'

# Construct full version string from these.
_VER = [_VER_MAJOR, _VER_MINOR]
if _VER_MICRO:
    _VER.append(_VER_MICRO)
if _VER_EXTRA:
    _VER.append(_VER_EXTRA)

__version__ = '.'.join(map(str, _VER))

setup(name='PyPCL',
      version=__version__,
      description='Python implementation of Point Cloud Library (PCL)',
      long_description='(see project homepage)',
      author='Jacob Zhong',
      author_email='cmpute@gmail.com',
      url='https://github.com/cmpute/pypcl',
      download_url='https://github.com/cmpute/pypcl/archive/master.zip',
      license='BSD-3-Clause',
      install_requires=['numpy', 'numpy-quaternion'],
      extras_require={
          'compress': ['python-lzf'],
          'search': ['nmslib'],
          'test': ['pytest'],
      },
      classifiers=[
          'Programming Language :: Python :: 3',
          'Programming Language :: Python :: 3.5',
          'License :: OSI Approved :: BSD License',
          'Operating System :: OS Independent',
          'Development Status :: 2 - Pre-Alpha',
          'Topic :: Scientific/Engineering'
      ],
      keywords=['pcl', 'pointcloud', 'numpy'],
      packages=find_packages()
     )
