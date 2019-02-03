import sys

try:
    from skbuild import setup
except ImportError:
    raise ImportError('scikit-build is required for installing')

setup(
    name="PyPCL",
    version="0.1.8",
    description="Cython bindings of Point Cloud Library (PCL)",
    long_description='(see project homepage)',
    author='Jacob Zhong',
    author_email='cmpute@gmail.com',
    url='https://github.com/cmpute/pypcl',
    download_url='https://github.com/cmpute/pypcl/archive/master.zip',
    license='BSD-3-Clause',
    packages=['pcl', 'pcl.common', 'pcl.filters', 'pcl.io', 'pcl.sample_consensus', 'pcl.visualization'],
    package_data={'pcl':['*.pxd', '*/*.pxd', '__init__.pxd', '_eigen.hpp']},
    install_requires=['numpy'],
    setup_requires=['cython', 'scikit-build'],
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
