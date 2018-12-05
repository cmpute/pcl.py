find_package(PCL REQUIRED COMPONENTS common)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cython_target(_pcl CXX)
add_library(_pcl MODULE ${_pcl})
python_extension_module(_pcl LINKED_MODULES_VAR linked_module_list)

install(TARGETS _pcl LIBRARY DESTINATION pcl)
