cmake_minimum_required(VERSION 3.24)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
    librmcs
    URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.3.0b1/librmcs-sdk-src-3.3.0-beta.1-debug.zip
    URL_HASH SHA256=446d632b23b652dd309f35075ef90928dee89f42e8ec71417fa2439445e0b4ff
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
    add_executable(yaw_pid_profile_test test/yaw_pid_profile.cpp)
    target_compile_options(yaw_pid_profile_test PRIVATE -UNDEBUG)
    add_test(NAME yaw_pid_profile COMMAND yaw_pid_profile_test)
    find_package(fmt REQUIRED)
    ament_auto_add_executable(yaw_pid_baseline_test test/yaw_pid_baseline.cpp)
    target_link_libraries(yaw_pid_baseline_test fmt::fmt)
    target_compile_options(yaw_pid_baseline_test PRIVATE -UNDEBUG)
    add_test(NAME yaw_pid_baseline COMMAND yaw_pid_baseline_test --ros-args --params-file
        ${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/deformable-infantry-omni-c.yaml)
    set_tests_properties(yaw_pid_baseline PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp/rmcs-pid-test-logs")
    ament_auto_add_executable(direct_yaw_sweep_test test/direct_yaw_sweep.cpp)
    target_link_libraries(direct_yaw_sweep_test fmt::fmt)
    target_compile_options(direct_yaw_sweep_test PRIVATE -UNDEBUG)
    add_test(NAME direct_yaw_sweep COMMAND direct_yaw_sweep_test
        --ros-args --params-file
        ${PROJECT_SOURCE_DIR}/test/direct_yaw_sweep.yaml)
    set_tests_properties(direct_yaw_sweep PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp/rmcs-sweep-test-logs")
endif()

ament_auto_package()
