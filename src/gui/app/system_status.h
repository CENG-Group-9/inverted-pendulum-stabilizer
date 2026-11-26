#pragma once
#include <chrono>
#include <cmath>

namespace pendulum::gui {
struct SystemStatus {
    double rod_angle_radians{0.0}; // 0 is upright
    double rod_angular_velocity_radians_per_second{0.0};
    double cart_position_meters{0.0}; // along the rail
    double cart_velocity_meters_per_second{0.0};
    double cart_acceleration_meters_per_second_squared{0.0};
    double time_seconds{0.0};
};

class StatusController {
  public:
    StatusController() : start_time_{std::chrono::steady_clock::now()} {
        current_status_ = get_dummy_sensor_readings();
    }

    void update_status() {
        current_status_ = is_demo_mode_ ? get_dummy_sensor_readings() : get_sensor_readings();
    }

    [[nodiscard]] const SystemStatus& current_status() const { return current_status_; }

  private:
    static constexpr double kDemoCartAmplitudeMeters = 0.35;
    static constexpr double kDemoCartAngularFrequencyRadiansPerSecond = 1.2;
    static constexpr double kDemoRodAmplitudeRadians = 1.0;
    static constexpr double kDemoRodAngularFrequencyRadiansPerSecond = 2.1;
    static constexpr double kDemoRodPhaseRadians = 2.0;

    bool is_demo_mode_{true};
    std::chrono::steady_clock::time_point start_time_{};
    SystemStatus current_status_{};

    [[nodiscard]] SystemStatus get_sensor_readings() const {
        return SystemStatus{}; // TODO: replace with telemetry from the serial source.
    }

    [[nodiscard]] SystemStatus get_dummy_sensor_readings() const {
        const double elapsed_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        const double cart_phase_radians = kDemoCartAngularFrequencyRadiansPerSecond * elapsed_seconds;
        const double rod_phase_radians = kDemoRodAngularFrequencyRadiansPerSecond * elapsed_seconds + kDemoRodPhaseRadians;

        SystemStatus status{};
        status.time_seconds = elapsed_seconds;
        status.cart_position_meters = kDemoCartAmplitudeMeters * std::sin(cart_phase_radians);
        status.cart_velocity_meters_per_second = kDemoCartAmplitudeMeters *
                                                 kDemoCartAngularFrequencyRadiansPerSecond *
                                                 std::cos(cart_phase_radians);
        status.cart_acceleration_meters_per_second_squared =
            -kDemoCartAmplitudeMeters * kDemoCartAngularFrequencyRadiansPerSecond *
            kDemoCartAngularFrequencyRadiansPerSecond * std::sin(cart_phase_radians);
        status.rod_angle_radians = kDemoRodAmplitudeRadians * std::sin(rod_phase_radians);
        status.rod_angular_velocity_radians_per_second = kDemoRodAmplitudeRadians *
                                                         kDemoRodAngularFrequencyRadiansPerSecond *
                                                         std::cos(rod_phase_radians);
        return status;
    }
};
} // namespace pendulum::gui
