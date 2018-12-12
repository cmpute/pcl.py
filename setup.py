import sys

try:
    from skbuild import setup
except ImportError:
    raise ImportError('scikit-build is required for installing')

setup(
    name="PyPCL",
    version="0.0.1",
    description="Minial PCL Binding",
    author='Jacob Zhong',
    license="MIT",
    packages=['pcl'],
    package_data={'pcl':['*.pxd', '*/*.pxd', '__init__.pxd']},
    install_requires=['cython', 'eigency'],
)
