cmake_minimum_required(VERSION 2.8.3)
project(lab_table_description)

# ROS 2
if("$ENV{ROS_VERSION}" EQUAL "2")
  cmake_policy(VERSION 3.5)

  # find dependencies
  find_package(ament_cmake REQUIRED)

  # Set install locations
  set(${PROJECT_NAME}_BIN_DESTINATION lib/${PROJECT_NAME})
  set(${PROJECT_NAME}_SHARE_DESTINATION share/${PROJECT_NAME})

  # Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

# ROS 1
else()
  find_package(catkin REQUIRED)

  catkin_package()

  if(CATKIN_ENABLE_TESTING)
  endif()

  # Set install locations
  set(${PROJECT_NAME}_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(${PROJECT_NAME}_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

# Install URDF and mesh files to share destination
install(DIRECTORY meshes urdf
  DESTINATION ${${PROJECT_NAME}_SHARE_DESTINATION}
)
