import sys
from skbuild import setup

setup(
    name="pcl_tests",
    version = "1.0",
    packages = ["test"],
    setup_requires=["pytest-runner"],
    tests_require=["pytest"],
    cmake_args=['-DSETUP_TEST:BOOL=ON']
)
