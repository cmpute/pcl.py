find_package(PCL REQUIRED COMPONENTS common)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add at last to make sure using the right python library
include_directories(BEFORE ${PYTHON_INCLUDE_DIR})

# support cython/c api
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# generate config file
configure_file(${CMAKE_CURRENT_LIST_DIR}/pcl_config.pxi.in
    ${CMAKE_CURRENT_LIST_DIR}/pcl_config.pxi)

add_cython_target(PointField CXX)
add_library(PointField MODULE ${PointField})
target_link_libraries(PointField ${PCL_LIBRARIES})
python_extension_module(PointField)
install(TARGETS PointField LIBRARY DESTINATION pcl)

add_cython_target(PointCloud CXX)
add_library(PointCloud MODULE ${PointCloud})
target_link_libraries(PointCloud ${PCL_LIBRARIES})
python_extension_module(PointCloud)
install(TARGETS PointCloud LIBRARY DESTINATION pcl)

# add submodules
macro(add_pcl_module mname)
    find_package(PCL REQUIRED COMPONENTS ${mname})

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    add_cython_target(_${mname} CXX)
    add_library(_${mname} MODULE ${_${mname}})
    target_link_libraries(_${mname} ${PCL_LIBRARIES})
    python_extension_module(_${mname})
    install(TARGETS _${mname} LIBRARY DESTINATION pcl/${mname})
endmacro()

add_subdirectory(common)
add_subdirectory(filters)
add_subdirectory(io)
add_subdirectory(visualization)
