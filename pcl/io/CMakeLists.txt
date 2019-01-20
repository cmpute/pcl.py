find_package(PCL REQUIRED COMPONENTS io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cython_target(_io CXX)
add_library(_io MODULE ${_io})
target_link_libraries(_io ${PCL_LIBRARIES})
python_extension_module(_io)
install(TARGETS _io LIBRARY DESTINATION pcl/io)
