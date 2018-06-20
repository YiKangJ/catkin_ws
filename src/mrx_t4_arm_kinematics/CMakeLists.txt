cmake_minimum_required(VERSION 2.8.3)
project(mrx_t4_arm_kinematics)

find_package(catkin REQUIRED
            cmake_modules
            geometry_msgs
            kdl_conversions
        )
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(cmake_modules REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrx_t4_arm_kinematics
  
  DEPENDS
    orocos_kdl
    kdl_conversions
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
    include
    ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/inverse_kinematics.cpp
    src/logger.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_kdl src/test_kdl.cpp)
target_link_libraries(test_kdl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
install(TARGETS test_kdl DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
