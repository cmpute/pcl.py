import sys
from skbuild import setup

setup(
    name="pcl_tests",
    version = "0.0",
    packages = ["test"],
    package_data={'test':['data/*.pcd']},
    setup_requires=["pytest-runner"],
    tests_require=["pytest"],
    cmake_args=['-DSETUP_TEST:BOOL=ON']
)
