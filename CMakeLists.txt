cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(carla_its_converter)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()


  if(${ROS_DISTRO} STREQUAL "foxy")
      add_compile_definitions(ROS_DISTRO_FOXY)
  endif()

  if(${ROS_DISTRO} STREQUAL "humble")
      add_compile_definitions(ROS_DISTRO_HUMBLE)
  endif()

  if(${ROS_DISTRO} STREQUAL "iron")
      add_compile_definitions(ROS_DISTRO_IRON)
  endif() 
  
  if(${ROS_DISTRO} STREQUAL "rolling")
      add_compile_definitions(ROS_DISTRO_ROLLING)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  # uncomment the following section in order to fill in
  # further dependencies manually.
  # find_package(<dependency> REQUIRED)  
  find_package(derived_object_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(shape_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_perception_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  find_package(carla_msgs REQUIRED)
  find_package(etsi_its_msgs REQUIRED)
  find_package(etsi_its_msgs_utils REQUIRED)
  find_package(perception_msgs REQUIRED)
  find_package(perception_msgs_utils REQUIRED)

  add_executable(${PROJECT_NAME}_node src/CarlaItsConverterNode.cpp)

  target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  ament_target_dependencies(${PROJECT_NAME}_node
    derived_object_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    shape_msgs
    tf2
    tf2_geometry_msgs
    tf2_perception_msgs
    tf2_ros

    carla_msgs
    etsi_its_msgs
    etsi_its_msgs_utils
    perception_msgs
    perception_msgs_utils
  )
  ament_export_dependencies(
    derived_object_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    shape_msgs
    tf2
    tf2_geometry_msgs
    tf2_perception_msgs
    tf2_ros

    carla_msgs
    etsi_its_msgs
    etsi_its_msgs_utils
    perception_msgs
    perception_msgs_utils
  )

  install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    #find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    #set(ament_cmake_cpplint_FOUND TRUE)
    #ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()