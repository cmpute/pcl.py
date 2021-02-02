find_package(PCL COMPONENTS io)

if (${PCL_IO_FOUND})
    message(STATUS "PCL io found, build pcl.io")

    add_cython_target(_io CXX)
    add_library(_io MODULE ${_io})
    target_link_libraries(_io ${PCL_LIBRARIES})
    python_extension_module(_io)
    install(TARGETS _io LIBRARY DESTINATION pcl/io)
else()
    message(STATUS "PCL io not found, skip pcl.io")
endif()
