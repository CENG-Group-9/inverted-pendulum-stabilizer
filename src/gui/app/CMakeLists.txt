# The executable. Intentionally thin: it creates the main window, wires the
# telemetry source to the view modules and does nothing else. Behavior belongs in
# core/ or in one of the view modules.

qt_add_executable(pendulum-gui main.cpp)

target_link_libraries(pendulum-gui PRIVATE
    pendulum_gui_core
    Qt6::Widgets
    Qt6::SerialPort)
