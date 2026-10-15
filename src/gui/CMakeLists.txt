cmake_minimum_required(VERSION 3.16)
project(pendulum-gui VERSION 1.0 LANGUAGES CXX)

# Enforce modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Automatically handle Qt boilerplate code
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find the user's local Qt installation
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort)

# List your C++ source files here
add_executable(pendulum-gui
    main.cpp
    # add other .cpp files here
)

# Link Qt libraries to your application
target_link_libraries(pendulum-gui PRIVATE 
    Qt6::Core 
    Qt6::Gui 
    Qt6::Widgets 
    Qt6::SerialPort
)
