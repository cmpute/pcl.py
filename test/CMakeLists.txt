find_package(PCL REQUIRED COMPONENTS common)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cython_target(_test_types CXX)
add_library(_test_types MODULE ${_test_types} _test_types_src.cxx)
python_extension_module(_test_types LINKED_MODULES_VAR linked_module_list)

install(TARGETS _test_types LIBRARY DESTINATION test)
