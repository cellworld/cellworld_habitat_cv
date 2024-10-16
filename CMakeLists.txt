cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 20)

project(habitat_cv
        VERSION 2019.1.0
        DESCRIPTION "A very opinionated open cv implementation for research"
        LANGUAGES CXX)

string(APPEND CMAKE_CXX_FLAGS " -DC_GNU64=400 -DOS_LINUX -no-pie -Wall -Wextra -pthread -Wno-deprecated-enum-enum-conversion")
string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")

####
#### OPTIONS
####

option(USE_FAKE_CAMERA OFF)
option(USE_CUDA OFF)
option(PLAN_B OFF)
option(USE_MCP2221 OFF)

####
#### DEPENDENCIES
####

find_package (Dependencies QUIET)

if (NOT ${Dependencies_FOUND})
    if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
        file(DOWNLOAD https://raw.githubusercontent.com/germanespinosa/dependencies/main/DependenciesConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
    endif()
    set(Dependencies_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    find_package (Dependencies REQUIRED)
endif()

####
#### END DEPENDENCIES
####

include_directories (include)

find_package (OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories (${OpenCV_INCLUDE_DIRS})

###
### MAIN LIBRARY SETUP
###

if (${USE_CUDA})
    add_definitions(-DUSE_CUDA=TRUE)
endif()

if (${USE_FAKE_CAMERA})
    message (STATUS "Using fake cameras")
    set( habitat_cv_camera_file src/fake_camera.cpp)
else()
    message (STATUS "Using real cameras")
    include_directories (/usr/local/xclib/inc/)
    set( habitat_cv_camera_file src/camera.cpp)
endif()


set( habitat_cv_files
        src/detection.cpp
        src/composite.cpp
        src/camera_configuration.cpp
        src/image.cpp
        src/video.cpp
        src/layout.cpp
        src/camera_array.cpp
        src/frame_rate.cpp
        src/layouts.cpp
        src/background.cpp
        src/cv_service.cpp
        ${habitat_cv_camera_file} src/tools/create_robot_occlusions.cpp)

add_library(habitat_cv ${habitat_cv_files})

install_git_dependency( Perf-analysis
        https://github.com/cellworld/dependency_perf-analysis
        CMAKE_PROJECT
        IMPORT_TARGETS perf-analysis
        INCLUDE_DIRECTORIES include)

if (${USE_MCP2221})
    install_git_dependency ( Mcp2221_lib
            https://github.com/cellworld/dependency_mcp2221
            CMAKE_PROJECT
            IMPORT_TARGETS
            mcp2221
            INCLUDE_DIRECTORIES include
            CONNECTED)
    add_definitions(-DUSE_SYNCHRONIZATION=TRUE)
    message(STATUS "Using MCP2221 synchronization")
endif()

install_git_dependency ( Pid_controller
        https://github.com/cellworld/cellworld_pid_controller
        CMAKE_PROJECT
        IMPORT_TARGETS
        controller
        CONNECTED)

install_git_dependency ( Robot_pid_lib
        https://github.com/cellworld/cellworld_robot_library
        CMAKE_PROJECT
        IMPORT_TARGETS
        gamepad_lib
        cellworld
        requests
        json-cpp
        params-cpp
        easy-tcp
        tcp_messages
        agent_tracking
        experiment_service
        robot_lib
        CONNECTED)

add_executable(agent_tracker
        src/tools/agent_tracker.cpp)

target_link_libraries(habitat_cv
        PRIVATE
        cellworld
        ${OpenCV_LIBS}
        agent_tracking
        robot_lib
        perf-analysis
        params-cpp)

if (${USE_MCP2221})
    target_link_libraries(habitat_cv
            PRIVATE
            mcp2221 )
endif()

target_link_libraries(agent_tracker habitat_cv )

add_executable(reverse_videos_logs
        src/tools/reverse_videos_logs.cpp)

target_link_libraries(reverse_videos_logs habitat_cv)


add_executable(control_frame_shift
        src/tools/control_frame_shift.cpp)

target_link_libraries(control_frame_shift habitat_cv)

add_executable(create_visibility
        src/tools/create_visibility.cpp)

target_link_libraries(create_visibility cellworld params-cpp)

add_executable(create_paths
        src/tools/create_paths.cpp)

target_link_libraries(create_paths cellworld params-cpp)

add_executable(create_robot_occlusions
        src/tools/create_robot_occlusions.cpp)

target_link_libraries(create_robot_occlusions cellworld params-cpp)

add_executable(create_predator_destinations
        src/tools/create_predator_destinations.cpp)

target_link_libraries(create_predator_destinations cellworld params-cpp)


add_executable(create_spawn_locations
        src/tools/create_spawn_locations.cpp)

target_link_libraries(create_spawn_locations cellworld params-cpp)

add_executable(homography src/tools/homography.cpp)
target_link_libraries(homography habitat_cv)


if (NOT ${USE_FAKE_CAMERA})

    add_executable(camera_config
            src/tools/camera_config.cpp
            src/frame_rate.cpp
            src/camera.cpp
            src/camera_array.cpp
            src/image.cpp
            src/video.cpp)


    target_link_libraries(camera_config
            habitat_cv
            cellworld
            params-cpp
            ${OpenCV_LIBS} )

    message (STATUS "Linking PIXCI libraries")
    target_link_libraries(camera_config
            /usr/local/xclib/lib/xclib_x86_64.a m -pthread)

    target_link_libraries(habitat_cv
            PRIVATE
            /usr/local/xclib/lib/xclib_x86_64.a m -pthread )
else()
    add_executable(camera_test
            src/tools/camera_config.cpp
            src/frame_rate.cpp
            src/fake_camera.cpp
            src/camera_array.cpp
            src/image.cpp
            src/video.cpp)

    target_link_libraries(camera_test
            cellworld
            ${OpenCV_LIBS} )

endif()
###
### TESTS
###
install_git_dependency( Catch
        https://github.com/cellworld/dependency_catch
        ADD_SUBDIRECTORY)
