cmake_minimum_required(VERSION 3.20)

project(impulse-dynamics)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Git QUIET)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/tmig/CMakeLists.txt")
    message(STATUS "TMIG submodule not found. Attempting to pull submodule...")

    if(GIT_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive --progress external/tmig
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE TMIG_SUBMODULE_RESULT
        )

        if(NOT TMIG_SUBMODULE_RESULT EQUAL "0")
            message(FATAL_ERROR
                "Failed to pull TMIG submodule.\n"
                "Try manually:\n"
                "git submodule update --init --recursive"
            )
        endif()
    else()
        message(FATAL_ERROR
            "Git not found and TMIG submodule is missing.\n"
            "Run manually:\n"
            "git submodule update --init --recursive"
        )
    endif()
endif()

add_subdirectory(external/tmig)

add_executable(impulse-dynamics
    src/main.cpp
    src/render/scene_renderer.cpp
    src/sim/world.cpp
    src/sim/snapshot.cpp
    src/sim/timeline.cpp
    src/sim/scenarios.cpp
    src/solvers/global_solver.cpp
    src/solvers/sequential_solver.cpp
)

set(CMAKE_BUILD_TYPE Release)
target_include_directories(impulse-dynamics PRIVATE include)
target_link_libraries(impulse-dynamics PRIVATE tmig)

# Symlink at repo root so clangd / C++ extension find the compilation database.
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(_compile_commands_link "${CMAKE_SOURCE_DIR}/compile_commands.json")
    if(EXISTS "${_compile_commands_link}")
        file(REMOVE "${_compile_commands_link}")
    endif()
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${_compile_commands_link}"
    )
endif()
