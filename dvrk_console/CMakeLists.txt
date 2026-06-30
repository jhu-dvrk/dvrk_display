cmake_minimum_required(VERSION 3.8)
project(dvrk_console)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crtk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvrk_data REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

add_library(dvrk_console_overlay
  src/dvrk_console/overlay.cpp
  src/dvrk_console/overlay_utils.cpp
  src/dvrk_console/overlay_components.cpp
)
target_include_directories(dvrk_console_overlay PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dvrk_console
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
)
target_compile_options(dvrk_console_overlay PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
  ${CAIRO_CFLAGS_OTHER}
  ${GTKMM_CFLAGS_OTHER}
)
ament_target_dependencies(dvrk_console_overlay
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(dvrk_console_overlay
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${GTKMM_LIBRARIES}
)

add_executable(stereo_display
  src/dvrk_console/main_stereo_display.cpp
)
add_executable(control_panel
  src/dvrk_console/main_control_panel.cpp
)

foreach(exe stereo_display control_panel)
  ament_target_dependencies(${exe}
    rclcpp
    sensor_msgs
    std_msgs
    crtk_msgs
    dvrk_data
  )
  target_include_directories(${exe} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dvrk_console
    ${GST_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${GTKMM_INCLUDE_DIRS}
  )
  target_compile_options(${exe} PRIVATE
    ${GST_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
    ${CAIRO_CFLAGS_OTHER}
    ${GTKMM_CFLAGS_OTHER}
  )
  target_link_libraries(${exe}
    ${GST_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${GTKMM_LIBRARIES}
    dvrk_console_overlay
  )
endforeach()

install(TARGETS
  stereo_display
  control_panel
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_calibrate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY share/config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
