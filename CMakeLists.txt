cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME gazebo_interface)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake

#add_definitions(-std=c++0x)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)



set(GAZEBO_INTERFACE_SOURCE_DIR
	src/source)
	
set(GAZEBO_INTERFACE_INCLUDE_DIR
	src/include
	)

set(GAZEBO_INTERFACE_HEADER_FILES
  src/include/
  )

find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions)



catkin_package(
#       INCLUDE_DIRS ${OKTODRIVER_INCLUDE_DIR}
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions
  )


include_directories(${GAZEBO_INTERFACE_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(gazebo_interface ${GAZEBO_INTERFACE_SOURCE_FILES} ${GAZEBO_INTERFACE_HEADER_FILES})
add_dependencies(gazebo_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_interface ${catkin_LIBRARIES})

add_executable(gazebo_battery_interface  src/source/gazebo_battery_interface.cpp)
add_dependencies(gazebo_battery_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_battery_interface gazebo_interface)
target_link_libraries(gazebo_battery_interface ${catkin_LIBRARIES})

add_executable(gazebo_imu_interface  src/source/gazebo_imu_interface.cpp)
add_dependencies(gazebo_imu_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_imu_interface gazebo_interface)
target_link_libraries(gazebo_imu_interface ${catkin_LIBRARIES})

add_executable(gazebo_camera_interface  src/source/gazebo_camera_interface.cpp)
add_dependencies(gazebo_camera_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_camera_interface gazebo_interface)
target_link_libraries(gazebo_camera_interface ${catkin_LIBRARIES})

add_executable(gazebo_ground_truth_interface  src/source/gazebo_ground_truth_interface.cpp)
add_dependencies(gazebo_ground_truth_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ground_truth_interface gazebo_interface)
target_link_libraries(gazebo_ground_truth_interface ${catkin_LIBRARIES})

add_executable(gazebo_command_interface  src/source/gazebo_command_interface.cpp)
add_dependencies(gazebo_command_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_command_interface gazebo_interface)
target_link_libraries(gazebo_command_interface ${catkin_LIBRARIES})
set_target_properties(gazebo_interface PROPERTIES LINKER_LANGUAGE CXX)

add_executable(gazebo_altitude_interface  src/source/gazebo_altitude_interface.cpp)
add_dependencies(gazebo_altitude_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_altitude_interface gazebo_interface)
target_link_libraries(gazebo_altitude_interface ${catkin_LIBRARIES})

add_executable(gazebo_speed_interface  src/source/gazebo_speed_interface.cpp)
add_dependencies(gazebo_speed_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_speed_interface gazebo_interface)
target_link_libraries(gazebo_speed_interface ${catkin_LIBRARIES})

add_executable(gazebo_flightstate_interface  src/source/gazebo_flightstate_interface.cpp)
add_dependencies(gazebo_flightstate_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_flightstate_interface gazebo_interface)
target_link_libraries(gazebo_flightstate_interface ${catkin_LIBRARIES})
