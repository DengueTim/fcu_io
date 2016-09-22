cmake_minimum_required(VERSION 2.8.3)
project(fcu_io)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  fcu_common
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
INCLUDE(FindPkgConfig)
pkg_check_modules(YAML yaml-cpp)

###################################
## catkin specific configuration ##
###################################
# 
# add_message_files(
#   FILES
#   Command.msg
#   ServoOutputRaw.msg
# )

add_service_files(
  FILES
  ParamFile.srv
  ParamGet.srv
  ParamSet.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS Boost Eigen yaml-cpp
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${YAML_INCLUDEDIR}
)

# fcu_io_node
add_executable(fcu_io_node
  src/fcu_io_node.cpp
  src/fcu_io.cpp
)
add_dependencies(fcu_io_node fcu_common_generate_messages_cpp)
add_dependencies(fcu_io_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcu_io_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS fcu_io_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

