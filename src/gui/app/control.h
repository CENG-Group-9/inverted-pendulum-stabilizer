#pragma once
#include <imgui.h>

namespace pendulum::gui {
    class ControlWindow
    {
        public:
        static void show_control_window(bool *p_open = NULL) 
        {
            static int opt_demo_mode = 0;
            static bool opt_demo_mode_changed = false;
            static ImGuiDockNodeFlags flags;

            ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

            // Refocus our window to minimize perceived loss of focus when changing mode (caused by the fact that each use a different window, which would not happen in a real app)
            if (opt_demo_mode_changed)
                ImGui::SetNextWindowFocus();

            ImGui::Begin("Examples: Dockspace", p_open, ImGuiWindowFlags_MenuBar);

            ImGui::SeparatorText("Inverted pendulum controllers");
            opt_demo_mode_changed = false;
            opt_demo_mode_changed |= ImGui::RadioButton("RL controller", &opt_demo_mode, 0);
            opt_demo_mode_changed |= ImGui::RadioButton("LQR controller", &opt_demo_mode, 1);
            opt_demo_mode_changed |= ImGui::RadioButton("Manual controller", &opt_demo_mode, 2);

            ImGui::End();  
        }
    };
}