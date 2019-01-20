find_package(PCL REQUIRED COMPONENTS sample_consensus)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cython_target(_sample_consensus CXX)
add_library(_sample_consensus MODULE ${_sample_consensus})
target_link_libraries(_sample_consensus ${PCL_LIBRARIES})
python_extension_module(_sample_consensus)
install(TARGETS _sample_consensus LIBRARY DESTINATION pcl/sample_consensus)
