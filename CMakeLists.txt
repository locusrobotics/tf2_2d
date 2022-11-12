cmake_minimum_required(VERSION 3.14.4)
project(tf2_2d)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror)
endif()

add_library(tf2_2d INTERFACE)
target_compile_features(tf2_2d INTERFACE cxx_std_14)
target_include_directories(tf2_2d INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tf2_2d INTERFACE
  Boost::boost
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS tf2_2d EXPORT tf2_2d-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export modern CMake target
ament_export_targets(tf2_2d-export)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # Exclude copyright check because it requires a CONTRIBUTING.md file
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_copyright")

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Conversion Tests
  ament_add_gtest(test_conversions
    test/test_conversions.cpp
  )
  target_link_libraries(test_conversions
    tf2_2d
  )

  # Rotation Tests
  ament_add_gtest(test_rotation
    test/test_rotation.cpp
  )
  target_link_libraries(test_rotation
    tf2_2d
  )

  # Transform Tests
  ament_add_gtest(test_transform
    test/test_transform.cpp
  )
  target_link_libraries(test_transform
    tf2_2d
  )

  # Vector2 Tests
  ament_add_gtest(test_vector2
    test/test_vector2.cpp
  )
  target_link_libraries(test_vector2
    tf2_2d
  )
endif()

ament_package()
