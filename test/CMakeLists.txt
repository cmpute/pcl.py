
find_package(PythonExtensions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS ${PCL_LIBRARY_DIRS})

# Add at last to make sure using the right python library
include_directories(BEFORE ${PYTHON_INCLUDE_DIR})

add_cython_target(_test_types CXX)
add_library(_test_types MODULE ${_test_types} _test_types_src.cxx)
target_link_libraries(_test_types ${PCL_LIBRARIES})
python_extension_module(_test_types)
install(TARGETS _test_types LIBRARY DESTINATION test)
