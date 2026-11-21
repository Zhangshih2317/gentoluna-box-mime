cmake_minimum_required(VERSION 3.16)
project(apex_wrist_sdk LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Vendor objects are built by build_sdk.sh with the stock build flags.
add_library(wrist_sdk SHARED sdk_wrist_runtime.cpp ${SDK_OBJECTS})
set_target_properties(wrist_sdk PROPERTIES OUTPUT_NAME GentoSDK)
target_include_directories(wrist_sdk PRIVATE .. ${GENTO_INCLUDES})
target_compile_definitions(wrist_sdk PRIVATE CMPL_LIN)
target_link_options(wrist_sdk PRIVATE "LINKER:-z,defs")
ament_target_dependencies(wrist_sdk rclcpp std_msgs)
target_link_libraries(wrist_sdk rt pthread)

# Maintenance host only links the SDK and waits for both wrist feedback topics.
# It exposes no arm command publisher and issues no servo/mode/position requests.
add_executable(verify_owner verify_owner.cpp)
target_include_directories(verify_owner PRIVATE ${GENTO_INCLUDES})
target_compile_definitions(verify_owner PRIVATE CMPL_LIN)
target_link_libraries(verify_owner wrist_sdk)
ament_target_dependencies(verify_owner rclcpp std_msgs)
