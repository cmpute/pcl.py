cmake_minimum_required(VERSION 3.12)

project(pcl_cython)

find_package(PythonExtensions REQUIRED)
find_package(Cython REQUIRED)
find_package(NumPy REQUIRED)
include_directories(${NumPy_INCLUDE_DIRS})

if(SETUP_TEST)
    add_subdirectory(test)
else()
    add_subdirectory(pcl)
endif()
