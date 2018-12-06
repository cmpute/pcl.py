cmake_minimum_required(VERSION 3.5.0)

project(python_pcl)

find_package(PythonExtensions REQUIRED)
find_package(Cython REQUIRED)

if(SETUP_TEST)
    add_subdirectory(test)
else()
    add_subdirectory(pcl)
endif()
