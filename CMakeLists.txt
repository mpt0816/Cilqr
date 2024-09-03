cmake_minimum_required(VERSION 3.5)
project(planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

#指定库路径
#file(GLOB_RECURSE Python2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
#指定头文件路径
set(Python2.7_INLCUDE_DIRS 
  "/usr/include/python2.7"
  )

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
  LIBRARIES planning
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Python2.7_INLCUDE_DIRS}
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
  algorithm/planning_node.cc

  algorithm/math/aabox2d.cpp
  algorithm/math/box2d.cpp
  algorithm/math/line_segment2d.cpp
  algorithm/math/math_utils.cpp
  algorithm/math/polygon2d.cpp
  algorithm/math/vec2d.cpp
  algorithm/math/linear_quadratic_regulator.cc

  algorithm/planner/dp_planner.cpp
  algorithm/planner/trajectory_planner.cpp

  algorithm/utils/discretized_trajectory.cpp
  algorithm/utils/environment.cpp
  algorithm/utils/discrete_points_math.cc
  
  algorithm/visualization/plot.cpp
  algorithm/visualization/color.cpp

  algorithm/ilqr/corridor.cc
  algorithm/ilqr/ilqr_optimizer.cc
  algorithm/ilqr/vehicle_model.cc
  algorithm/ilqr/tracker.cc)

add_executable(${PROJECT_NAME}_node  main.cc ${SRC})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
)
