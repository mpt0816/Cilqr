cmake_minimum_required(VERSION 3.5)
project(planning)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  CenterLinePoint.msg
  CenterLine.msg
  DynamicObstacle.msg
  DynamicObstacles.msg
  Obstacles.msg
  DynamicTrajectoryPoint.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

option(WITH_HSL "Compile solver with coinhsl" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

set(SRC
  src/planning_node.cc
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/trajectory_planner.cpp
  src/discretized_trajectory.cpp
  src/environment.cpp
  src/dp_planner.cpp
  src/visualization/plot.cpp
  src/visualization/color.cpp)

add_executable(${PROJECT_NAME}_node  src/main.cc ${SRC})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
