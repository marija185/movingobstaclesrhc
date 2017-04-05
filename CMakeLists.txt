cmake_minimum_required(VERSION 2.4.6)
project(movingobstaclesrhc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(cspacevoroandrea REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

find_package(Boost REQUIRED COMPONENTS signals)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
  tf
)

add_message_files(
  FILES
  Coil.msg
  Moving.msg
)

add_service_files(
  FILES
  ReqRobStop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
  INCLUDE_DIRS include
  LIBRARIES movingobstaclesrhc
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${cspacevoroandrea_INCLUDE_DIRS}
  include
)

add_library(movingobstaclesrhc src/mainspoj.cpp src/WorkHorse.cpp src/DynamicWindow.cpp src/DStar.cpp src/GridMap.cpp src/Planner.cpp)
add_executable(spoj src/mainspoj.cpp src/WorkHorse.cpp src/DynamicWindow.cpp src/DStar.cpp src/GridMap.cpp src/Planner.cpp)
#target_link_libraries(spoj ${catkin_LIBRARIES} ${cspacevoroandrea_LIBRARIES})

add_dependencies(spoj movingobstaclesrhc_generate_messages_cpp)

target_link_libraries(spoj movingobstaclesrhc ${catkin_LIBRARIES} ${cspacevoroandrea_LIBRARIES})
#target_link_libraries(spoj movingobstaclesrhc ${catkin_LIBRARIES})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#ärosbuild_add_boost_directories()
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_find_ros_package(cspacevoronoi2p5D)
#now ${pkg_PACKAGE_PATH} is set


#add_subdirectory(cspacevoronoi2p5D/src)
#add_subdirectory(/home/marija/programi/ros/ros_workspace/cspacevoronoi2p5D/src) nejde tak
#rosbuild_add_executable(talker src/talker.cpp)
#rosbuild_add_executable(listener src/sub.cpp)
#rosbuild_add_executable(thermalmapper src/main.cpp)
#rosbuild_add_executable(vozi src/mainback.cpp)
#target_link_libraries(spoj movingobstaclesrhc) #Do not use target_link_libraries() to link against a library built by another ROS package
#rosbuild_add_executable(novi src/novi.cpp)
#rosbuild_add_executable(idi src/mainidi.cpp)
#rosbuild_add_executable(robot src/mainlaser.cpp)
#rosbuild_link_boost(spoj signals filesystem regex system)


#comment this line if not using electric
#find_package(OpenCV REQUIRED)
#define some target ...
#comment this too if not using electric
#target_link_libraries(spoj ${OpenCV_LIBS} newmat)
#uncomment this line if not using electric
#target_link_libraries(spoj newmat)
