cmake_minimum_required(VERSION 3.8)
project(ur10e_ik_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(fcl REQUIRED)

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
find_package(Eigen3 REQUIRED)

add_library(ur10e_kinematics src/ur10e_kinematics.cpp)
target_include_directories(ur10e_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ur10e_kinematics Eigen3::Eigen)
target_compile_features(ur10e_kinematics PUBLIC cxx_std_17)

add_executable(fk_test src/fk_test.cpp)
target_link_libraries(fk_test ur10e_kinematics Eigen3::Eigen)

install(TARGETS ur10e_kinematics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS fk_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

add_executable(ik_round_trip_test src/ik_round_trip_test.cpp)
target_link_libraries(ik_round_trip_test ur10e_kinematics Eigen3::Eigen)
install(TARGETS ik_round_trip_test DESTINATION lib/${PROJECT_NAME})

add_executable(ik_control_node src/ik_control_node.cpp)
target_link_libraries(ik_control_node ur10e_kinematics Eigen3::Eigen)
ament_target_dependencies(ik_control_node
  rclcpp sensor_msgs geometry_msgs trajectory_msgs std_msgs std_srvs tf2)
install(TARGETS ik_control_node DESTINATION lib/${PROJECT_NAME})

add_executable(fcl_test src/fcl_test.cpp)
target_link_libraries(fcl_test fcl)
install(TARGETS fcl_test DESTINATION lib/${PROJECT_NAME})

add_executable(collision_test src/collision_test.cpp)
target_link_libraries(collision_test ur10e_kinematics fcl Eigen3::Eigen)
target_include_directories(collision_test PRIVATE include)
install(TARGETS collision_test DESTINATION lib/${PROJECT_NAME})

add_executable(rrt_test src/rrt_test.cpp)
target_link_libraries(rrt_test ur10e_kinematics fcl Eigen3::Eigen)
target_include_directories(rrt_test PRIVATE include)
install(TARGETS rrt_test DESTINATION lib/${PROJECT_NAME})

target_link_libraries(ik_control_node ur10e_kinematics fcl)

ament_package()
