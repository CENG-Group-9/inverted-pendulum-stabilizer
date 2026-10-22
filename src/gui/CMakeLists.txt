# The GUI is its own project: configure and build it from this directory.
#   cmake --preset debug
#   cmake --build --preset debug
#
# Module layout (dependency direction is strictly downward):
#
#   app/             executable, wires everything together
#   twin/           3D digital twin           (not created yet)
#   plots/          2D sensor plots           (not created yet)
#   control_panel/  parameter controls        (not created yet)
#   core/           data, units, parsers, interfaces - testable, no widgets
#
# View modules may depend on core/ only. They must never depend on each other.

cmake_minimum_required(VERSION 3.25)

project(pendulum-gui VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Needed by `clang-tidy -p build/debug` (see AGENTS.md).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/pendulum_warnings.cmake)

find_package(Qt6 6.3 REQUIRED COMPONENTS Core Gui Widgets SerialPort)
if(BUILD_TESTING)
    find_package(Qt6 6.3 REQUIRED COMPONENTS Test)
endif()

qt_standard_project_setup()

add_subdirectory(core)
add_subdirectory(app)

# ---------------------------------------------------------------------------
# Developer checks (AGENTS.md: format and tidy before finishing a change)
#   cmake --build --preset debug --target format-check
#   cmake --build --preset debug --target tidy-check
#   cmake --build --preset debug --target check
#
# Sources are globbed so new files and new module directories are covered
# automatically; the build/ tree (moc, autogen) is excluded.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE pendulum_gui_all_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
list(FILTER pendulum_gui_all_sources EXCLUDE REGEX "/build/")

# Only .cpp files are passed to clang-tidy: headers have no compile command.
set(pendulum_gui_tidy_sources ${pendulum_gui_all_sources})
list(FILTER pendulum_gui_tidy_sources INCLUDE REGEX "\\.cpp$")

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
if(CLANG_FORMAT_EXECUTABLE)
    add_custom_target(format-check
        COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror ${pendulum_gui_all_sources}
        COMMENT "Checking C++ formatting with clang-format"
        VERBATIM)
else()
    message(STATUS "clang-format not found: 'format-check' target is disabled")
endif()

find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy)
if(CLANG_TIDY_EXECUTABLE)
    add_custom_target(tidy-check
        COMMAND ${CMAKE_COMMAND}
                "-DCLANG_TIDY_EXECUTABLE=${CLANG_TIDY_EXECUTABLE}"
                "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
                "-DFILES=${pendulum_gui_tidy_sources}"
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/run_clang_tidy.cmake
        COMMENT "Running clang-tidy on GUI sources"
        VERBATIM)
else()
    message(STATUS "clang-tidy not found: 'tidy-check' target is disabled")
endif()

if(TARGET format-check AND TARGET tidy-check)
    add_custom_target(check
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        DEPENDS format-check tidy-check
        COMMENT "Running format, tidy and unit tests"
        VERBATIM)
endif()
