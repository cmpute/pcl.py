import glob

try:
    from skbuild import setup
except ImportError:
    raise ImportError('scikit-build is required for installing')

setup(
    name="pcl-py",
    version="0.2.11",
    description="Cython bindings of Point Cloud Library (PCL)",
    long_description='(see project homepage)',
    author='Jacob Zhong',
    author_email='cmpute@gmail.com',
    url='https://github.com/cmpute/pcl.py',
    download_url='https://github.com/cmpute/pcl.py/archive/master.zip',
    license='BSD-3-Clause',
    packages=['pcl', 'pcl.common', 'pcl.filters', 'pcl.io', 'pcl.visualization'],
    install_requires=['numpy>=1.11'],
    setup_requires=['cython>=0.29', 'scikit-build'],
    extras_require={'test': ['pytest']},
    package_data={'pcl': [p.lstrip("pcl/") for p in (glob.glob("pcl/*.pyi") + glob.glob("pcl/**/*.pyi"))]},
    classifiers=[
        'Programming Language :: C++',
        'Programming Language :: Cython',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Development Status :: 2 - Pre-Alpha',
        'Topic :: Scientific/Engineering'
    ],
    keywords=['pcl', 'pointcloud', 'numpy', 'cython', 'binding'],
)
