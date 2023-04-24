cmake_minimum_required(VERSION 3.10.2)
project(robotiq_85_description)

#ROS 2
if("$ENV{ROS_VERSION}" EQUAL "2")

  find_package(ament_cmake REQUIRED)
  install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
  install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")
  ament_package()

#ROS 1
else()

  find_package(
    catkin REQUIRED
  )
  
  install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
  install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")
  
  catkin_package()

endif()

