find_package(PCL REQUIRED COMPONENTS visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cython_target(_visualization CXX)
add_library(_visualization MODULE ${_visualization})
target_link_libraries(_visualization ${PCL_LIBRARIES})
python_extension_module(_visualization)
install(TARGETS _visualization LIBRARY DESTINATION pcl/visualization)
