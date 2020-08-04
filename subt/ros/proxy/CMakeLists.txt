cmake_minimum_required(VERSION 3.5.1)
project(proxy)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    subt_ign
    roscpp
)

### find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ros_proxy ros_proxy_node.cc)
add_dependencies(ros_proxy ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_proxy ${catkin_LIBRARIES} -lzmq)

add_executable(ros_proxy_teambase ros_proxy_teambase.cc)
add_dependencies(ros_proxy_teambase ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_proxy_teambase ${catkin_LIBRARIES} -lzmq)

###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Mark executables and/or libraries for installation
install(TARGETS ros_proxy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
