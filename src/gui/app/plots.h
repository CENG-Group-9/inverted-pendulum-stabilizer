#pragma once
#include "system_status.h"
#include <cmath>
#include <cstddef>
#include <deque>
#include <imgui.h>
#include <implot.h>
#include <iostream>
#include <string>
#include <vector>

namespace pendulum::gui {
enum class GraphOption { CartPosition, CartVelocity, RodAngle, RodAngularVelocity, Time };

class PlotsWindow {
  public:
    // About 30 seconds of history at 100 Hz. Older samples are dropped.
    static constexpr std::size_t kMaximumSamples = 3000;

    static inline unsigned int num_measurements;
    static inline std::deque<SystemStatus> status_data;
    static inline bool cart_position_active = false;
    static inline bool cart_velocity_active = false;
    static inline bool rod_angle_active = false;
    static inline bool rod_angular_velocity_active = false;

    static void show_plots_window(const SystemStatus& current_system_status,
                                  bool* p_open = nullptr) {
        record_sample(current_system_status);

        if (ImGui::Begin("Plots", p_open)) {
            ImGui::Checkbox("cart position(m) vs time(s)", &cart_position_active);
            ImGui::Checkbox("cart velocity(m/s) vs time(s)", &cart_velocity_active);
            ImGui::Checkbox("rod angle(rad) vs time(s)", &rod_angle_active);
            ImGui::Checkbox("rod angular velocity (rad/s) vs time", &rod_angular_velocity_active);

            if (cart_position_active) {
                show_plot(GraphOption::CartPosition, "Cart position", "Cart position (m)");
            }
            if (cart_velocity_active) {
                show_plot(GraphOption::CartVelocity, "Cart velocity", "Cart velocity(m/s)");
            }
            if (rod_angle_active) {
                show_plot(GraphOption::RodAngle, "Rod angle", "Rod angle(rad)");
            }
            if (rod_angular_velocity_active) {
                show_plot(GraphOption::RodAngularVelocity, "rod_angular_velocity",
                          "rod angular velocity(rad/s)");
            }
        }

        ImGui::End();
    }

    // Skips repeated frames (same timestamp) and keeps the history bounded.
    static void record_sample(const SystemStatus& new_status) {
        const bool is_new_sample =
            status_data.empty() || status_data.back().time_seconds != new_status.time_seconds;
        if (!is_new_sample) {
            return;
        }

        status_data.push_back(new_status);
        if (status_data.size() > kMaximumSamples) {
            status_data.pop_front();
        }
    }

    static void show_plot(GraphOption option, const std::string& graph_name,
                          const std::string& label_id) {
        const std::vector<double> data_column = get_column(option);
        const std::vector<double> time_column = get_column(GraphOption::Time);
        if (!data_column.empty() && ImPlot::BeginPlot(graph_name.c_str(), ImVec2(-1, 300))) {
            ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine(label_id.c_str(), time_column.data(), data_column.data(),
                             static_cast<int>(data_column.size()));
            ImPlot::EndPlot();
        }
    }

    static void demo_status_data() {
        constexpr double kSamplePeriodSeconds = 1.0 / 60.0;
        constexpr double kDemoAmplitudeMeters = 0.25;
        constexpr double kDemoAngularFrequency = 2.0;

        SystemStatus new_status{};
        new_status.time_seconds =
            status_data.empty() ? 0.0 : status_data.back().time_seconds + kSamplePeriodSeconds;
        new_status.cart_position_meters =
            kDemoAmplitudeMeters * std::sin(kDemoAngularFrequency * new_status.time_seconds);
        record_sample(new_status);
    }

    static std::vector<double> get_column(GraphOption option) {
        std::vector<double> data;
        data.reserve(status_data.size());

        switch (option) {
        case GraphOption::CartPosition:
            for (const auto& element : status_data) {
                data.push_back(element.cart_position_meters);
            }
            break;
        case GraphOption::CartVelocity:
            for (const auto& element : status_data) {
                data.push_back(element.cart_velocity_meters_per_second);
            }
            break;
        case GraphOption::RodAngle:
            for (const auto& element : status_data) {
                data.push_back(element.rod_angle_radians);
            }
            break;
        case GraphOption::RodAngularVelocity:
            for (const auto& element : status_data) {
                data.push_back(element.rod_angular_velocity_radians_per_second);
            }
            break;
        case GraphOption::Time:
            for (const auto& element : status_data) {
                data.push_back(element.time_seconds);
            }
            break;
        default:
            std::cerr << "ERROR::GETCOLUMN_GIVEN_OPTION_IS_NOT_VALID\n";
            break;
        }

        return data;
    }
};

} // namespace pendulum::gui
