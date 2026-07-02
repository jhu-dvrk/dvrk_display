cmake_minimum_required(VERSION 3.8)
project(dvrk_data)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
pkg_check_modules(GTK REQUIRED gtk+-3.0)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

add_library(dvrk_data
  src/config.cpp
  src/collection_config.cpp
  src/tags.cpp
  src/gst_utils.cpp
)
target_include_directories(dvrk_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
target_compile_options(dvrk_data PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)
target_link_libraries(dvrk_data
  ${JSONCPP_LIBRARIES}
  ${GST_LIBRARIES}
)
ament_target_dependencies(dvrk_data
  ament_index_cpp
)

add_executable(record
  src/record/main.cpp
  src/record/app_data.cpp
  src/record/gst_helpers.cpp
  src/record/video_stream.cpp
  src/record/audio_stream.cpp
  src/record/ui.cpp
  src/record/ros_node.cpp
)
ament_target_dependencies(record
  rclcpp
  std_msgs
  rosbag2_cpp
)
target_include_directories(record PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(record
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

add_executable(video_tag
  src/video_tag/tag_main.cpp
  src/video_tag/tag_ui.cpp
)
target_include_directories(video_tag PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_tag
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  gstvideo-1.0
  dvrk_data
)

add_executable(video_latency
  src/video_latency/main.cpp
)
target_include_directories(video_latency PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_latency
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  gstvideo-1.0
  dvrk_data
)

add_executable(stereo_source
  src/stereo_source/main.cpp
)
ament_target_dependencies(stereo_source
  rclcpp
)
target_include_directories(stereo_source PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_options(stereo_source PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
)
target_link_libraries(stereo_source
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

add_executable(stereo_alignment
  src/stereo_alignment/main.cpp
)
ament_target_dependencies(stereo_alignment
  rclcpp
)
target_include_directories(stereo_alignment PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_options(stereo_alignment PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
)
target_link_libraries(stereo_alignment
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

add_executable(video_configurator
  src/video_configurator/video_configurator.cpp
)
target_include_directories(video_configurator PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_configurator
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS dvrk_data
  EXPORT export_dvrk_data
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  record
  stereo_source
  stereo_alignment
  video_tag
  video_latency
  video_configurator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/synthetic_source.py
  scripts/gstreamer_ar.py
  scripts/opencv_ar.py
  scripts/smoke_test_stack
  scripts/extract
  scripts/encord_to_tags
  scripts/stereo_alignment_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_dvrk_data HAS_LIBRARY_TARGET)
ament_export_libraries(dvrk_data)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp)

ament_package()
