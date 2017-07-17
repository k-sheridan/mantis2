cmake_minimum_required(VERSION 2.8.3)
project(mantis2)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cmake_modules
  tf
  message_filters
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/mantis2
	${catkin_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}
 	${GLUT_INCLUDE_DIRS}
 	${GLEW_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
# CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS opencv
)

add_library(mantis2_params include/mantis2/Mantis2Parameters.h)
set_target_properties(mantis2_params PROPERTIES LINKER_LANGUAGE CXX)

add_library(mantis2_types include/mantis2/Mantis2Types.h)
set_target_properties(mantis2_types PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(mantis2_types ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_library(yaw_markov include/mantis2/YawMarkov.cpp)
set_target_properties(yaw_markov PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(yaw_markov ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} mantis2_params mantis2_types)

add_executable(mantis2_node src/mantis2_node.cpp)
target_link_libraries(mantis2_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})