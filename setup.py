import sys
from skbuild import setup

setup(
    name="pcl-test",
    version="0.0.1",
    description="minial pcl binding",
    author='The scikit-build team',
    license="MIT",
    packages=['pcl'],
    install_requires=['cython'],
)
