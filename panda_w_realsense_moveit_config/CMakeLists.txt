# cmake_minimum_required(VERSION 3.1.3)
# project(panda_w_realsense_moveit_config)

# find_package(catkin REQUIRED COMPONENTS
#              rospy
#              actionlib
# 	           actionlib_msgs
#              trajectory_msgs
#              sensor_msgs
#              control_msgs
# )

# find_package(Boost REQUIRED COMPONENTS
#              system 
#              thread
# )

# catkin_package(
#               CATKIN_DEPENDS
#               actionlib
#               actionlib_msgs
#               trajectory_msgs
#               rospy
#               sensor_msgs
#               DEPENDS Boost
# )

# include_directories(
#   # include
#   ${catkin_INCLUDE_DIRS}
# )

# file(GLOB_RECURSE moveit_config RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.py)

# add_executable(moveit_config
#   ${moveit_config}
# )
# set_target_properties(moveit_config_wrapper PROPERTIES LINKER_LANGUAGE CXX)


# # add_dependencies(moveit_config 
# #   sensor_msgs_gencpp
# #   ${PROJECT_NAME}_gencfg
# #   ${PROJECT_NAME}_gencpp
# # )

# target_link_libraries(moveit_config
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN "setup_assistant.launch" EXCLUDE)
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.1.3)
project(panda_w_realsense_moveit_config)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    trajectory_msgs
    sensor_msgs
    control_msgs
)

find_package(Boost REQUIRED COMPONENTS
    system 
    thread
)

catkin_package(
    CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    trajectory_msgs
    rospy
    sensor_msgs
    DEPENDS Boost
)

file(GLOB_RECURSE moveit_config RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.py)

add_custom_target(moveit_config ALL
    SOURCES ${moveit_config}
)

add_dependencies(moveit_config ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "setup_assistant.launch" EXCLUDE
)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
