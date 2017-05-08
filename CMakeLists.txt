cmake_minimum_required(VERSION 2.8.3)
project(jrk_hardware)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  serial
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jrk_hardware
)

###########
## Build ##
###########

include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(jrk_serial src/jrk_serial.cpp)
target_link_libraries(jrk_serial ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/jrk_hardware.cpp)
target_link_libraries(${PROJECT_NAME} jrk_serial ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/jrk_hardware_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# add_executable(${PROJECT_NAME}_tests tests/jrk_control_tests.cpp)
# add_dependencies(${PROJECT_NAME}_tests ${PROJECT_NAME})
# target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} # ${PROJECT_NAME}_tests
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jrk_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
