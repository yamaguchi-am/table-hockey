cmake_minimum_required(VERSION 2.6)

project(hockey)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(gflags)
find_package(GTest REQUIRED)


set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")

if (NOT gflags_FOUND)
  add_definitions(-D gflags=google)
endif (NOT gflags_FOUND)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(cv_util
  cv_util.cc
)
target_link_libraries(cv_util
  ${OpenCV_LIBS}
)

add_library(calib
  calib.cc
)
target_link_libraries(calib
  ${OpenCV_LIBS}
)

add_library(color_detector
  color_detector.cc
)
target_link_libraries(color_detector
  ${OpenCV_LIBS}
  cv_util
)

add_library(ball_detector
  ball_detector.cc
)
target_link_libraries(ball_detector
  ${OpenCV_LIBS}
  cv_util
)

add_library(arm
  arm.cc
)
target_link_libraries(arm
  ${OpenCV_LIBS}
  cminpack
)

add_library(motion
  motion.cc
)
target_link_libraries(motion
  ${OpenCV_LIBS}
)

add_executable(motion_test
  motion_test.cc
)
target_link_libraries(motion_test
  motion
  GTest::GTest
  GTest::Main
)

add_library(motion_estimator
  motion_estimator.cc
)

add_subdirectory(kondo-ics)
add_subdirectory(third_party/cminpack)

add_executable(main
  main.cc
  time.cc
  kondo_servo_driver.cc
)
target_link_libraries(main
    ${OpenCV_LIBS}
    arm
    ball_detector
    calib
    color_detector
    cv_util
    kondo_ics
    motion
)
