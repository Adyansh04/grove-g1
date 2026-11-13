cmake_minimum_required(VERSION 3.22)
project(sam3_inference LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sam3_inference_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ZLIB REQUIRED)
# The engine needs CUDA and TensorRT; without them only the CPU parts (tokenizer, tensor types,
# request checks) and their tests build. TensorRT ships no CMake package: its libraries come from
# NVIDIA's apt repository.
find_package(CUDAToolkit QUIET)
find_library(NVINFER_LIBRARY nvinfer)
find_library(NVONNXPARSER_LIBRARY nvonnxparser)
if(CUDAToolkit_FOUND AND NVINFER_LIBRARY AND NVONNXPARSER_LIBRARY)
  set(SAM3_WITH_ENGINE ON)
else()
  set(SAM3_WITH_ENGINE OFF)
  message(WARNING "sam3_inference: CUDA or TensorRT not found, building the CPU parts only")
endif()

function(sam3_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_20)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endfunction()

add_library(sam3_inference_cpu SHARED
  src/clip_tokenizer.cpp
  src/request_checks.cpp
  src/tensor_types.cpp
)
target_include_directories(sam3_inference_cpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(sam3_inference_cpu PRIVATE ZLIB::ZLIB)
sam3_target_defaults(sam3_inference_cpu)
install(TARGETS sam3_inference_cpu
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(SAM3_WITH_ENGINE)
  add_library(sam3_inference_core SHARED src/sam3_engine.cpp)
  target_link_libraries(sam3_inference_core
    PUBLIC sam3_inference_cpu opencv_core
    PRIVATE opencv_imgproc CUDA::cudart ${NVINFER_LIBRARY} ${NVONNXPARSER_LIBRARY}
  )
  sam3_target_defaults(sam3_inference_core)

  # A plugin loaded by the component container or the generated executable: nothing links it, so
  # its dependencies stay private and it is not exported.
  add_library(sam3_segmentation_component SHARED src/sam3_segmentation_node.cpp)
  target_link_libraries(sam3_segmentation_component PRIVATE
    sam3_inference_core
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    rclcpp_components::component
    ${rcl_interfaces_TARGETS}
    ${sam3_inference_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  )
  sam3_target_defaults(sam3_segmentation_component)
  # Images keep arriving while a request runs.
  rclcpp_components_register_node(sam3_segmentation_component
    PLUGIN "sam3_inference::Sam3SegmentationNode"
    EXECUTABLE sam3_segmentation_node
    EXECUTOR MultiThreadedExecutor
  )
  install(TARGETS sam3_inference_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS sam3_segmentation_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch resources DESTINATION share/${PROJECT_NAME})
# Documentation of the export: it runs in a venv with torch and Meta's sam3, not in this image.
install(FILES scripts/export_sam3.py DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sam3_cpu test/test_clip_tokenizer.cpp test/test_cpu_parts.cpp)
  target_link_libraries(test_sam3_cpu sam3_inference_cpu)
  target_compile_definitions(test_sam3_cpu PRIVATE
    SAM3_VOCABULARY="${CMAKE_CURRENT_SOURCE_DIR}/resources/bpe_simple_vocab_16e6.txt.gz")
  sam3_target_defaults(test_sam3_cpu)

  if(SAM3_WITH_ENGINE)
    # Needs a GPU and the engines: built for manual runs, never registered with ctest.
    add_executable(check_sam3_engine test/check_sam3_engine.cpp)
    target_link_libraries(check_sam3_engine PRIVATE
      sam3_inference_core opencv_imgproc opencv_imgcodecs)
    sam3_target_defaults(check_sam3_engine)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)
ament_package()
