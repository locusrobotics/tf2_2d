cmake_minimum_required(VERSION 2.8.3)
project(tf2_2d)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs)

# TODO(swilliams) Add support for python2 and python3

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf2
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

# Header-only package. Nothing to build.

# Python bindings
add_library(tf2_2d SHARED
  src/bindings.cpp
)
target_include_directories(tf2_2d
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)
target_link_libraries(tf2_2d
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
set_target_properties(tf2_2d PROPERTIES SUFFIX .so)
set_target_properties(tf2_2d PROPERTIES PREFIX "")

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# TODO(swilliams) I need to figure out where and how to install the tf2_2d.so file that contains the python bindings

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
