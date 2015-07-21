cmake_minimum_required(VERSION 2.8.3)
project(collision_recovery)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        nav_core
        costmap_2d
        actionlib_msgs
        std_msgs
        pluginlib
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES collision_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
        nav_core
)

add_library(${PROJECT_NAME} src/collision_recovery.cpp)
add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)

install(TARGETS collision_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES collision_recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
