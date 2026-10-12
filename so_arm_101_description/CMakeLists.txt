cmake_minimum_required(VERSION 3.8)
project(so_arm_101_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(st3215_driver
            src/SCS.cpp
            src/SCSCL.cpp
            src/SCSerial.cpp
            src/SMS_STS.cpp
            src/SMSBL.cpp
            src/SMSCL.cpp            
            )

set_target_properties(st3215_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
            
#  target_include_directories(st3215_driver PUBLIC
#     include
# )

target_include_directories(st3215_driver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/st3215_driver>
        $<INSTALL_INTERFACE:include/st3215_driver>
)

add_executable(motor_node
              src/motor_node.cpp)

target_link_libraries(motor_node
               st3215_driver)

install(
  TARGETS st3215_driver
  EXPORT export_so_arm_101_description
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/st3215_driver/
  DESTINATION include/st3215_driver
)

# Install other directories
install(
  DIRECTORY meshes config launch rviz urdf include
  DESTINATION share/${PROJECT_NAME})

  ament_target_dependencies(motor_node 
                            rclcpp std_msgs
                            geometry_msgs
                            sensor_msgs
                            tf2_ros
                            tf2_geometry_msgs)

install(TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME})
  
#<exec_depend>ros2launch</exec_depend>

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_so_arm_101_description HAS_LIBRARY_TARGET)

ament_package()
