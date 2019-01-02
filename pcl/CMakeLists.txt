find_package(PCL REQUIRED COMPONENTS common io visualization)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add at last to make sure using the right python library
include_directories(BEFORE ${PYTHON_INCLUDE_DIR})

# support cython/c api
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR})

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

add_cython_target(Visualizer CXX)
add_library(Visualizer MODULE ${Visualizer})
target_link_libraries(Visualizer ${PCL_LIBRARIES})
python_extension_module(Visualizer)
install(TARGETS Visualizer LIBRARY DESTINATION pcl)

add_subdirectory(io)
