import sys

try:
    from skbuild import setup
except ImportError:
    raise ImportError('scikit-build is required for installing')

setup(
    name="pcl-py",
    version="0.2.0",
    description="Cython bindings of Point Cloud Library (PCL)",
    long_description='(see project homepage)',
    author='Jacob Zhong',
    author_email='cmpute@gmail.com',
    url='https://github.com/cmpute/pcl.py',
    download_url='https://github.com/cmpute/pcl.py/archive/master.zip',
    license='BSD-3-Clause',
    packages=['pcl', 'pcl.common', 'pcl.filters', 'pcl.io', 'pcl.visualization'],
    package_data={'pcl':['*.pxd', '*/*.pxd', '__init__.pxd', 'include/*']},
    install_requires=['numpy'],
    setup_requires=['cython>=0.28', 'scikit-build'],
    extras_require={'test': ['pytest']},
    classifiers=[
        'Programming Language :: C++',
        'Programming Language :: Cython',
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Development Status :: 2 - Pre-Alpha',
        'Topic :: Scientific/Engineering'
    ],
    keywords=['pcl', 'pointcloud', 'numpy', 'cython', 'binding'],
)
