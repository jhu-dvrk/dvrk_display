cmake_minimum_required(VERSION 3.8)
project(dvrk_display)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CAIRO REQUIRED cairo)

add_library(dvrk_display_lib
  src/common/config.cpp
  src/common/gst_utils.cpp
  src/common/ros_utils.cpp
  src/dvrk_display/overlay.cpp
)
target_include_directories(dvrk_display_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/dvrk_display>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
target_compile_options(dvrk_display_lib PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
  ${CAIRO_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)
target_link_libraries(dvrk_display_lib
  ${JSONCPP_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${GST_LIBRARIES}
  gstvideo-1.0
)
ament_target_dependencies(dvrk_display_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  ament_index_cpp
)

add_executable(stereo
  src/dvrk_display/stereo.cpp
)
target_link_libraries(stereo
  dvrk_display_lib
)
ament_target_dependencies(stereo
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
)

add_executable(mono
  src/dvrk_display/mono.cpp
)
target_link_libraries(mono
  dvrk_display_lib
)
ament_target_dependencies(mono
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
)

install(TARGETS
  stereo
  mono
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
