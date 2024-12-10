cmake_minimum_required(VERSION 3.0.2)
project(tf2_2d)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_geometry_msgs
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

# On ARM, the use of Eigen 3.4 with g++11 (both the versions in Jammy) causes a warning, which in turn causes
# the build to fail. Details are here: https://gitlab.com/libeigen/eigen/-/issues/2326. We can suppress the warning
# for ARM builds here.
if(Eigen3_VERSION VERSION_EQUAL 3.4 AND
   CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 11.0 AND
   CMAKE_LIBRARY_ARCHITECTURE STREQUAL "aarch64-linux-gnu")
    add_compile_options(-Wno-class-memaccess)
endif()

# Header-only package. Nothing to build.

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  find_package(roslint REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Conversion Tests
  catkin_add_gtest(test_conversions
    test/test_conversions.cpp
  )
  add_dependencies(test_conversions
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_conversions
    ${catkin_LIBRARIES}
  )

  # Rotation Tests
  catkin_add_gtest(test_rotation
    test/test_rotation.cpp
  )
  add_dependencies(test_rotation
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_rotation
    ${catkin_LIBRARIES}
  )

  # Transform Tests
  catkin_add_gtest(test_transform
    test/test_transform.cpp
  )
  add_dependencies(test_transform
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_transform
    ${catkin_LIBRARIES}
  )

  # Vector2 Tests
  catkin_add_gtest(test_vector2
    test/test_vector2.cpp
  )
  add_dependencies(test_vector2
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_vector2
    ${catkin_LIBRARIES}
  )
endif()
