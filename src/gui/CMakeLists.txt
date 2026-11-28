cmake_minimum_required(VERSION 3.25)
project(pendulum-gui LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIB_SERIALPORT REQUIRED IMPORTED_TARGET libserialport)

add_executable(pendulum-gui app/main.cpp
    app/stb_image.cpp
    libs/imgui/imgui.cpp libs/imgui/imgui_draw.cpp libs/imgui/imgui_widgets.cpp
    libs/imgui/imgui_tables.cpp libs/imgui/imgui_demo.cpp
    libs/imgui/backends/imgui_impl_glfw.cpp libs/imgui/backends/imgui_impl_opengl3.cpp
    libs/implot/implot.cpp libs/implot/implot_items.cpp libs/implot/implot_demo.cpp
    libs/glad/glad.c)

target_compile_definitions(pendulum-gui PRIVATE
    PENDULUM_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/app/shaders"
    PENDULUM_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/app/assets")

target_include_directories(pendulum-gui PRIVATE libs libs/glad libs/imgui libs/implot libs/imgui/backends libs/KHR)
target_link_libraries(pendulum-gui PRIVATE glfw OpenGL::GL assimp::assimp
    Threads::Threads PkgConfig::LIB_SERIALPORT)
