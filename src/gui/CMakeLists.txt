cmake_minimum_required(VERSION 3.25)
project(pendulum-gui LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(pendulum-gui app/main.cpp
    libs/imgui/imgui.cpp libs/imgui/imgui_draw.cpp libs/imgui/imgui_widgets.cpp
    libs/imgui/imgui_tables.cpp libs/imgui/imgui_demo.cpp
    libs/imgui/backends/imgui_impl_glfw.cpp libs/imgui/backends/imgui_impl_opengl3.cpp
    libs/implot/implot.cpp libs/implot/implot_items.cpp libs/implot/implot_demo.cpp
    libs/glad/glad.c)

target_include_directories(pendulum-gui PRIVATE libs libs/glad libs/imgui libs/implot libs/imgui/backends libs/KHR)
target_link_libraries(pendulum-gui PRIVATE glfw OpenGL::GL)
