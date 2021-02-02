find_package(PCL COMPONENTS filters)

add_cython_target(_filters CXX)
add_library(_filters MODULE ${_filters})
target_link_libraries(_filters ${PCL_LIBRARIES})
python_extension_module(_filters)
install(TARGETS _filters LIBRARY DESTINATION pcl/filters)
