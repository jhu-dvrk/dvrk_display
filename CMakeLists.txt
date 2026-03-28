cmake_minimum_required(VERSION 3.8)
project(dvrk_stereo_viewer)

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

add_library(dvrk_stereo_viewer_lib
  src/common/config.cpp
)
target_include_directories(dvrk_stereo_viewer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_options(dvrk_stereo_viewer_lib PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
)
target_link_libraries(dvrk_stereo_viewer_lib
  ${JSONCPP_LIBRARIES}
)
ament_target_dependencies(dvrk_stereo_viewer_lib
  ament_index_cpp
)

add_executable(dvrk_stereo_viewer
  src/dvrk_stereo_viewer/main.cpp
  src/dvrk_stereo_viewer/overlay.cpp
)
ament_target_dependencies(dvrk_stereo_viewer
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
)
target_include_directories(dvrk_stereo_viewer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
)
target_compile_options(dvrk_stereo_viewer PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
  ${CAIRO_CFLAGS_OTHER}
)
target_link_libraries(dvrk_stereo_viewer
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${CAIRO_LIBRARIES}
  dvrk_stereo_viewer_lib
)

install(TARGETS
  dvrk_stereo_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_configurator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
