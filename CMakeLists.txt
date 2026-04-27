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
find_package(data_collection REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

add_library(dvrk_display
  src/common/config.cpp
)
target_include_directories(dvrk_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
target_compile_options(dvrk_display PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)
target_link_libraries(dvrk_display
  ${JSONCPP_LIBRARIES}
  ${GST_LIBRARIES}
)
ament_target_dependencies(dvrk_display
  ament_index_cpp
)

add_executable(stereo
  src/dvrk_display/main_stereo.cpp
  src/dvrk_display/overlay.cpp
  src/dvrk_display/overlay_utils.cpp
  src/dvrk_display/overlay_components.cpp
)
ament_target_dependencies(stereo
  rclcpp
  sensor_msgs
  std_msgs
  data_collection
)
target_include_directories(stereo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
)
target_compile_options(stereo PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
  ${CAIRO_CFLAGS_OTHER}
  ${GTKMM_CFLAGS_OTHER}
)
target_link_libraries(stereo
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${GTKMM_LIBRARIES}
  dvrk_display
)

install(TARGETS
  stereo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
