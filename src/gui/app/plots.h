#pragma once
#include "system_status.h"
#include <imgui.h>
#include <vector>
#include <implot.h>
#include <cmath>
#include <string>
#include <iostream>

namespace pendulum::gui {
    enum class GraphOption
    {
        CART_POSITION,
        CART_VELOCITY,
        ROD_ANGLE,
        ROD_ANGULAR_VELOCITY,
        TIME
    };

    class PlotsWindow
    {
        public:
        static inline unsigned int num_measurements;
        static inline std::vector<SystemStatus> status_data;
        static inline bool cart_position_active = false;
        static inline bool cart_velocity_active = false;
        static inline bool rod_angle_active = false;
        static inline bool rod_angular_velocity_active = false; 

        static void show_plots_window(SystemStatus current_system_status, bool* p_open = nullptr)
        {   
            status_data.push_back(current_system_status);
            
            if (ImGui::Begin("Plots", p_open))
            {
                ImGui::Checkbox("cart position(m) vs time(s)", &cart_position_active);
                ImGui::Checkbox("cart velocity(m/s) vs time(s)", &cart_velocity_active);
                ImGui::Checkbox("rod angle(rad) vs time(s)", &rod_angle_active);
                ImGui::Checkbox("rod angular velocity (rad/s) vs time", &rod_angular_velocity_active);

                if (cart_position_active)
                {
                    show_plot(GraphOption::CART_POSITION, "Cart position", "Cart position (m)");
                }
                if (cart_velocity_active)
                {
                    show_plot(GraphOption::CART_VELOCITY, "Cart velocity", "Cart velocity(m/s)");
                }
                if (rod_angle_active)
                {
                    show_plot(GraphOption::ROD_ANGLE, "Rod angle", "Rod angle(rad)");
                }
                if (rod_angular_velocity_active)
                {
                    show_plot(GraphOption::ROD_ANGULAR_VELOCITY, "rod_angular_velocity", "rod angular velocity(rad/s)");
                }
            }

            ImGui::End();
        }

        static void show_plot(GraphOption option, std::string graph_name, std::string label_id) 
        {
            
            const std::vector<double> data_column = get_column(option);
            const std::vector<double> time_column = get_column(GraphOption::TIME);
            if (!data_column.empty() && ImPlot::BeginPlot(graph_name.c_str(), ImVec2(-1, 300)))
            {
                ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine(label_id.c_str(),time_column.data(), data_column.data(), static_cast<int>(data_column.size()));
                ImPlot::EndPlot();
            }
        }

        static void demo_status_data()
        {
            constexpr double kSamplePeriodSeconds = 1.0 / 60.0;
            constexpr double kDemoAmplitudeMeters = 0.25;
            constexpr double kDemoAngularFrequency = 2.0;

            SystemStatus new_status{};
            new_status.time = static_cast<double>(status_data.size()) * kSamplePeriodSeconds;
            new_status.cartPosition = kDemoAmplitudeMeters * std::sin(kDemoAngularFrequency * new_status.time);
            status_data.push_back(new_status);
        }

        static std::vector<double> get_column(GraphOption option)
        {
            std::vector<double> data;

            switch (option) {
                case GraphOption::CART_POSITION:
                    for (auto elm : status_data)
                    {
                        data.push_back(elm.cartPosition);
                    }
                    break;
                case GraphOption::CART_VELOCITY:
                    for (auto elm : status_data)
                    {
                        data.push_back(elm.cartVelocity);
                    }
                    break;
                case GraphOption::ROD_ANGLE:
                    for (auto elm : status_data)
                    {
                        data.push_back(elm.rodAngle);
                    }
                    break;
                case GraphOption::ROD_ANGULAR_VELOCITY:
                    for (auto elm : status_data)
                    {
                        data.push_back(elm.rodAngularVelocity);
                    }
                    break;
                case GraphOption::TIME:
                    for (auto elm: status_data)
                    {
                        data.push_back(elm.time);
                    }
                    break;
                default:
                    std::cerr << "ERROR::GETCOLUMN_GIVEN_OPTION_IS_NOT_VALID\n";
                    break;
            }

            return data;
        }
    };
    
}