cmake_minimum_required(VERSION 2.8.3)
project(mrx_t4_arm_kinematics_moveit)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")
# from: https://blog.csdn.net/qq_38288618/article/details/78488298

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    moveit_core
    urdf
    tf_conversions
    mrx_t4_arm_kinematics
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mrx_t4_arm_kinematics_moveit
  src/mrx_t4_arm_kinematics_moveit.cpp
  src/kinematics_logger.cpp
)

target_link_libraries(mrx_t4_arm_kinematics_moveit
  ${catkin_LIBRARIES}
)

install(TARGETS mrx_t4_arm_kinematics_moveit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_gtest(load_moveit_plugin_test
  test/load_moveit_plugin_test.cpp
)

target_link_libraries(load_moveit_plugin_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(mrx_t4_arm_kinematics_moveit_test
  test/mrx_t4_arm_kinematics_moveit_test.cpp
)

target_link_libraries(mrx_t4_arm_kinematics_moveit_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(configuration_comparator_test
  test/configuration_comparator_test.cpp
)

target_link_libraries(configuration_comparator_test 
        ${catkin_LIBRARIES})
