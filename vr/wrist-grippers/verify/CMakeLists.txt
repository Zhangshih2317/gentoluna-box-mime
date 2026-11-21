cmake_minimum_required(VERSION 3.16)
project(luna_wrist_verify LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(marvin_msgs REQUIRED)
set(GENTO_INCLUDE "/usr/local/include/gentosdk" CACHE PATH "Installed Gento SDK headers")
find_library(GENTO_LIBRARY NAMES GentoSDK REQUIRED)

add_executable(protocol_test ../test_protocol.cpp)
target_compile_options(protocol_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
enable_testing()
add_test(NAME protocol COMMAND protocol_test)
add_executable(manual_test test_manual.cpp)
target_compile_options(manual_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME manual COMMAND manual_test)

add_library(wrist_link_check SHARED adapter_link.cpp)
target_include_directories(wrist_link_check PRIVATE ..
    "${GENTO_INCLUDE}/L1Robot" "${GENTO_INCLUDE}/Common" "${GENTO_INCLUDE}/L0Control"
    "${GENTO_INCLUDE}/Kinematics" "${GENTO_INCLUDE}/Kinematics/ArmKinematics"
    "${GENTO_INCLUDE}/Kinematics/BaseMath" "${GENTO_INCLUDE}/Kinematics/DynaIdent"
    "${GENTO_INCLUDE}/Kinematics/KineCommon" "${GENTO_INCLUDE}/Kinematics/MotionPlanner"
    "${GENTO_INCLUDE}/Kinematics/SkyeBodyKinematics")
target_compile_definitions(wrist_link_check PRIVATE CMPL_LIN)
target_link_options(wrist_link_check PRIVATE "LINKER:-z,defs")
target_link_libraries(wrist_link_check ${GENTO_LIBRARY} pthread)
ament_target_dependencies(wrist_link_check rclcpp std_msgs)

add_executable(ros_adapter_test test_ros_adapter.cpp)
get_target_property(wrist_includes wrist_link_check INCLUDE_DIRECTORIES)
add_executable(wrist_manual EXCLUDE_FROM_ALL ../manual_control.cpp)
target_include_directories(wrist_manual PRIVATE ${wrist_includes})
target_compile_definitions(wrist_manual PRIVATE CMPL_LIN)
target_link_libraries(wrist_manual ${GENTO_LIBRARY} pthread)
ament_target_dependencies(wrist_manual rclcpp std_msgs)
add_executable(inspect_wrist EXCLUDE_FROM_ALL inspect_wrist.cpp)
target_include_directories(inspect_wrist PRIVATE ${wrist_includes})
target_compile_definitions(inspect_wrist PRIVATE CMPL_LIN)
target_link_libraries(inspect_wrist ${GENTO_LIBRARY} pthread)
ament_target_dependencies(inspect_wrist rclcpp std_msgs)
target_include_directories(ros_adapter_test PRIVATE ${wrist_includes})
target_compile_definitions(ros_adapter_test PRIVATE CMPL_LIN)
target_compile_options(ros_adapter_test PRIVATE -UNDEBUG)
ament_target_dependencies(ros_adapter_test rclcpp std_msgs)
add_test(NAME ros_adapter COMMAND ros_adapter_test)
add_test(NAME trigger_without_position COMMAND ros_adapter_test --no-position)
add_test(NAME wrist_recovery COMMAND ros_adapter_test --recovery)
set_tests_properties(ros_adapter trigger_without_position wrist_recovery PROPERTIES TIMEOUT 120
    ENVIRONMENT "ROS_DOMAIN_ID=93;ROS_LOCALHOST_ONLY=1")

if(NOT OWNER_SOURCE)
    return()
endif()
add_executable(wrist_reference_node EXCLUDE_FROM_ALL
    "${OWNER_SOURCE}/marvin_robot_node.cpp"
    "${OWNER_SOURCE}/marvin_robot.cpp")
target_compile_definitions(wrist_reference_node PRIVATE CMPL_LIN)
target_include_directories(wrist_reference_node PRIVATE ..
    "${GENTO_INCLUDE}/L1Robot" "${GENTO_INCLUDE}/Common" "${GENTO_INCLUDE}/L0Control"
    "${GENTO_INCLUDE}/Kinematics" "${GENTO_INCLUDE}/Kinematics/ArmKinematics"
    "${GENTO_INCLUDE}/Kinematics/BaseMath" "${GENTO_INCLUDE}/Kinematics/DynaIdent"
    "${GENTO_INCLUDE}/Kinematics/KineCommon" "${GENTO_INCLUDE}/Kinematics/MotionPlanner"
    "${GENTO_INCLUDE}/Kinematics/SkyeBodyKinematics")
target_link_libraries(wrist_reference_node ${GENTO_LIBRARY} pthread rt)
ament_target_dependencies(wrist_reference_node rclcpp sensor_msgs geometry_msgs std_msgs
    marvin_msgs tf2 std_srvs)
