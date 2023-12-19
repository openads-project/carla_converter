cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(carla_its_converter)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  # set compile definitions for ROS2
  add_compile_definitions(MODE_ROS2)

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
  find_package(perception_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_perception_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(shape_msgs REQUIRED)
  find_package(carla_msgs REQUIRED)


  add_executable(${PROJECT_NAME}_node src/CarlaItsConverterNode.cpp)

  target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  ament_target_dependencies(${PROJECT_NAME}_node
    derived_object_msgs
    geometry_msgs
    nav_msgs
    perception_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_perception_msgs
    tf2_ros
    shape_msgs
    carla_msgs
  )
  ament_export_dependencies(
    derived_object_msgs
    geometry_msgs
    nav_msgs
    perception_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_perception_msgs
    tf2_ros
    shape_msgs
    carla_msgs
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

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)
  # set compile definitions for ROS1
  add_compile_definitions(MODE_ROS1)

  if(${ROS_DISTRO} STREQUAL "kinetic")
      add_compile_definitions(ROS_DISTRO_KINETIC)
  endif()

  if(${ROS_DISTRO} STREQUAL "noetic")
      add_compile_definitions(ROS_DISTRO_NOETIC)
  endif()
  
  ## Compile as C++11, supported in ROS Kinetic and newer
  add_compile_options(-std=c++11)

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    derived_object_msgs
    geometry_msgs
    perception_msgs
    roscpp
    roslaunch
    std_msgs
    tf2
    tf2_perception_msgs
    tf2_ros
    carla_msgs
  )


  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES hx_testmanager
    CATKIN_DEPENDS roscpp std_msgs derived_object_msgs perception_msgs tf2_perception_msgs tf2_ros tf2 geometry_msgs
  #  DEPENDS system_lib
  )

  ###########
  ## Build ##
  ###########

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(${PROJECT_NAME}_node src/CarlaItsConverterNode.cpp)
  ## EXECUTABLES END ##

  ## Rename C++ executable without prefix
  ## The above recommended prefix causes long target names, the following renames the
  ## target back to the shorter version for ease of user use
  ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
  # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

  ## TARGET PROPERTIES START ##

  #set_target_properties(${PROJECT_NAME}_SampleNodelet PROPERTIES OUTPUT_NAME SampleNodelet PREFIX lib)
  set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME CarlaItsConverterNode)
  ## TARGET PROPERTIES END ##

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  ## DEPENDENCIES START ##

  #add_dependencies(${PROJECT_NAME}_SampleNodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  ## DEPENDENCIES END ##

  ## Specify libraries to link a library or executable target against
  # target_link_libraries(${PROJECT_NAME}_node
  #   ${catkin_LIBRARIES}
  # )

  ## LINKING START ##

  #target_link_libraries(${PROJECT_NAME}_SampleNodelet ${catkin_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
  ## LINKING END ##

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
  # install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  # )
  ## INSTALL START ##
  install(TARGETS 
      ${PROJECT_NAME}_node
      #${PROJECT_NAME}_SampleNodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  ## INSTALL END ##


  ## Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  ## Mark other files for installation (e.g. launch and bag files, etc.)
  # install(FILES
  #   # myfile1
  #   # myfile2
  #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # )
  install(FILES
    #nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


  #############
  ## Testing ##
  #############

  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_hx_testmanager.cpp)
  #if(CATKIN_ENABLE_TESTING)
  #  find_package(rostest REQUIRED)
  #  add_rostest_gtest(test_ika_dogm test/ika_dogm.test test/UnitTest.cpp)
  #  catkin_add_gtest(test_ika_dogm test/UnitTest.cpp)
  #  target_link_libraries(test_ika_dogm ${catkin_LIBRARIES} ${PROJECT_NAME}_dogm_creation)
  #endif()
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
endif()

