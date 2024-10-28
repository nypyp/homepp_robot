cmake_minimum_required(VERSION 3.8)
project(yolov8_ros_tensorrt LANGUAGES CXX CUDA)

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
# list(REMOVE_ITEM CUDA_LIBRARIES Threads::Threads dl /usr/lib/aarch64-linux-gnu/librt.so)
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# OpenCV
find_package(OpenCV 4.5 REQUIRED)
message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

list(APPEND INCLUDE_DIRS
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        include
        )

list(APPEND ALL_LIBS
        ${CUDA_LIBRARIES}
        ${CUDA_LIB_DIR}
        ${OpenCV_LIBRARIES}
        ${TensorRT_LIBRARIES}
        )

include_directories(${INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()

add_executable(yolov8_ros_tensorrt_node 
        src/yolov8_ros_tensorrt_node.cpp
        )

ament_target_dependencies(yolov8_ros_tensorrt_node 
        rclcpp
        rclcpp_lifecycle
        lifecycle_msgs
        sensor_msgs 
        std_msgs
        cv_bridge
        yolov8_msgs
        )

target_link_directories(yolov8_ros_tensorrt_node PUBLIC ${ALL_LIBS})
target_link_libraries(yolov8_ros_tensorrt_node nvinfer nvinfer_plugin cudart ${OpenCV_LIBS})


install(TARGETS
  yolov8_ros_tensorrt_node
  DESTINATION lib/${PROJECT_NAME}
)


