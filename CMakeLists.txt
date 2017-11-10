cmake_minimum_required(VERSION 2.8.3)
project(code_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED
    roscpp
    std_msgs
    )

#set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polynomial cv_utils
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
     )
include_directories("include")

add_library(polynomial STATIC
    src/math_utils/Polynomial.cpp)
target_link_libraries(polynomial ${Boost_LIBRARIES} )

add_executable(poly_test
    src/poly_test.cpp
    src/math_utils/Polynomial.cpp)
target_link_libraries(poly_test ${Boost_LIBRARIES} )

add_library(cv_utils STATIC
    src/cv_utils.cc
    )
target_link_libraries(cv_utils ${Boost_LIBRARIES} )
