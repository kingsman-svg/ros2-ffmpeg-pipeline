cmake_minimum_required(VERSION 3.8)
project(video_pipeline)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(video_pipeline_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
  libavformat libavcodec libavutil libswscale libswresample
)

add_library(ffmpeg_video STATIC
  src/media_reader.cpp
  src/media_decoder.cpp
  src/video_encoder.cpp
  src/image_converter.cpp
  src/audio_resampler.cpp
)
target_include_directories(ffmpeg_video PUBLIC include)
target_link_libraries(ffmpeg_video PUBLIC PkgConfig::FFMPEG)
set_target_properties(ffmpeg_video PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_library(h264_packet_queue STATIC src/h264_packet_queue.cpp)
target_include_directories(h264_packet_queue PUBLIC include)
ament_target_dependencies(h264_packet_queue video_pipeline_msgs)
target_link_libraries(h264_packet_queue Threads::Threads)
set_target_properties(h264_packet_queue PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_library(bitrate_controller STATIC src/bitrate_controller.cpp)
target_include_directories(bitrate_controller PUBLIC include)
set_target_properties(bitrate_controller PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_executable(ffmpeg_video_probe src/ffmpeg_video_probe.cpp)
target_link_libraries(ffmpeg_video_probe PRIVATE ffmpeg_video)
set_target_properties(ffmpeg_video_probe PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_executable(ffmpeg_audio_probe src/ffmpeg_audio_probe.cpp)
target_link_libraries(ffmpeg_audio_probe PRIVATE ffmpeg_video)
set_target_properties(ffmpeg_audio_probe PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_executable(video_file_source_node nodes/video_file_source_node.cpp)
ament_target_dependencies(video_file_source_node rclcpp rcl_interfaces sensor_msgs std_msgs)
target_link_libraries(video_file_source_node ffmpeg_video)
set_target_properties(video_file_source_node PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_executable(h264_encoder_node nodes/h264_encoder_node.cpp)
ament_target_dependencies(h264_encoder_node rclcpp rcl_interfaces sensor_msgs std_msgs video_pipeline_msgs)
target_link_libraries(h264_encoder_node ffmpeg_video h264_packet_queue bitrate_controller)
set_target_properties(h264_encoder_node PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

add_executable(h264_decoder_node nodes/h264_decoder_node.cpp)
ament_target_dependencies(h264_decoder_node rclcpp rcl_interfaces sensor_msgs std_msgs video_pipeline_msgs)
target_link_libraries(h264_decoder_node ffmpeg_video h264_packet_queue)
set_target_properties(h264_decoder_node PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

install(TARGETS video_file_source_node h264_encoder_node h264_decoder_node
  ffmpeg_video_probe ffmpeg_audio_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(bitrate_controller_test test/bitrate_controller_test.cpp)
  target_link_libraries(bitrate_controller_test bitrate_controller)
  add_test(NAME bitrate_controller_test COMMAND bitrate_controller_test)
endif()

ament_package()
