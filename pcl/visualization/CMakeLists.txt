find_package(PCL COMPONENTS visualization)

if (${PCL_VISUALIZATION_FOUND})
    message(STATUS "PCL visualization found, build pcl.visualization")

    add_cython_target(_visualization CXX)
    add_library(_visualization MODULE ${_visualization})
    target_link_libraries(_visualization ${PCL_LIBRARIES})
    python_extension_module(_visualization)
    install(TARGETS _visualization LIBRARY DESTINATION pcl/visualization)
else()
    message(STATUS "PCL visualization not found, skip pcl.visualization")
endif()
