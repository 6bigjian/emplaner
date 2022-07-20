cmake_minimum_required(VERSION 3.0.2)
project(emplaner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sleipnir_msgs
  geometry_msgs
  routing_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_extension
  tf2_ros
  tf2_geometry_msgs
)


set(src_file
# src/xmldoc/XmlDocument.cpp

src/main.cpp
src/global_date.cpp

src/global/hdmap_building.cpp
src/global/global_planning.cpp

src/dynamic/dynamic_path.cpp

src/control/verhic_control.cpp

src/obstacle/obstacle.cpp

src/QuadProg/smooth.cpp
src/QuadProg/qpoasessolver.cpp
src/QuadProg/generateSTgraph.cpp
)

SET(CMAKE_BUILD_TYPE DEBUG) 


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES emplaner
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

link_directories("/usr/local/lib")

include_directories(
 include
  ${catkin_INCLUDE_DIRS}

  "/usr/local/include"
)

add_executable(emplaner_node ${src_file})
target_link_libraries(emplaner_node ${catkin_LIBRARIES} -lxerces-c)
target_link_libraries(emplaner_node libqpOASES.a)
