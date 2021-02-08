cmake_minimum_required(VERSION 2.8.3)
project(flir_ros)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#set(OpenCV2_DIR "/usr/local/include/")
find_package(OpenCV 3.4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	cv_bridge
	image_transport
	image_geometry
	sensor_msgs
)

add_service_files(
	FILES
)

add_message_files(
	FILES
)

#generate_messages(
#	DEPENDENCIES
#	std_msgs
#)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	/usr/local/include
)

catkin_package(
	CATKIN_DEPENDS message_runtime
   INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

## FLIR TARGET DETECTOR

## Declare a C++ executable
add_executable(flir_node src/flir_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(flir_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(flir_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	/usr/local/lib/libthermalgrabber.so
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dji_flir dji_flir_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_flir.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
