cmake_minimum_required(VERSION 3.12)

project(pcl_cython)

find_package(PythonExtensions REQUIRED)
find_package(Cython REQUIRED)
find_package(NumPy REQUIRED)
include_directories(${NumPy_INCLUDE_DIRS})

if(DEFINED ENV{BUILD_COV})
  add_compile_definitions(CYTHON_TRACE=1)
  set(CYTHON_FLAGS "-X linetrace=True")
endif()

if(SETUP_TEST)
  add_subdirectory(test)
else()
  add_subdirectory(pcl)
endif()
