# Pure logic for the GUI: data types, units, parsers, mode state.
# Must stay free of widgets, 3D and serial hardware so it can be unit tested
# without a display or a device. Everything else may depend on this; this must
# depend on nothing but QtCore and the standard library.

add_library(pendulum_gui_core STATIC
    control_mode.cpp
    control_mode.h)

target_include_directories(pendulum_gui_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pendulum_gui_core PUBLIC Qt6::Core pendulum_warnings)

if(BUILD_TESTING)
    qt_add_executable(test_control_mode tests/test_control_mode.cpp)
    target_link_libraries(test_control_mode PRIVATE pendulum_gui_core Qt6::Test)

    add_test(NAME control_mode COMMAND test_control_mode)
endif()
