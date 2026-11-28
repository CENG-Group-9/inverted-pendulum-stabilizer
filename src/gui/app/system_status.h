#pragma once
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include <libserialport.h>

namespace pendulum::gui {
struct SystemStatus {
    double rod_angle_radians{0.0}; // 0 is upright
    double rod_angular_velocity_radians_per_second{0.0};
    double cart_position_meters{0.0}; // along the rail
    double cart_velocity_meters_per_second{0.0};
    double cart_acceleration_meters_per_second_squared{0.0};
    double time_seconds{0.0};
};

inline constexpr std::string_view kDefaultSerialDevicePath = "/dev/ttyUSB0";

// One CSV line from the Arduino: cart position, cart velocity, rod angle, rod angular
// velocity. Returns nullopt when the line is malformed.
[[nodiscard]] inline std::optional<SystemStatus> parse_telemetry_line(std::string_view line) {
    SystemStatus status{};
    const std::array<double*, 4> fields{
        &status.cart_position_meters,
        &status.cart_velocity_meters_per_second,
        &status.rod_angle_radians,
        &status.rod_angular_velocity_radians_per_second,
    };

    std::size_t cursor = 0;
    for (std::size_t field_index = 0; field_index < fields.size(); ++field_index) {
        const bool is_last_field = field_index + 1 == fields.size();
        const std::size_t comma_position = line.find(',', cursor);
        if (!is_last_field && comma_position == std::string_view::npos) {
            return std::nullopt;
        }

        const std::size_t token_end = is_last_field ? line.size() : comma_position;
        const std::string_view token = line.substr(cursor, token_end - cursor);
        const auto [parse_end, parse_error] =
            std::from_chars(token.data(), token.data() + token.size(), *fields[field_index]);
        if (parse_error != std::errc{} || parse_end != token.data() + token.size()) {
            return std::nullopt;
        }
        cursor = token_end + 1;
    }
    return status;
}

// Reads newline-terminated CSV telemetry from a USB serial port on its own thread.
class SerialTelemetrySource {
  public:
    explicit SerialTelemetrySource(std::string device_path)
        : device_path_{std::move(device_path)},
          reader_{[this](std::stop_token stop_token) { run(stop_token); }} {}

    [[nodiscard]] std::optional<SystemStatus> latest_telemetry() const {
        const std::scoped_lock lock{telemetry_mutex_};
        return latest_telemetry_;
    }

    [[nodiscard]] bool is_connected() const { return is_connected_.load(); }

  private:
    static constexpr unsigned int kReadTimeoutMilliseconds = 100;
    static constexpr std::size_t kMaximumPendingBytes = 512;
    static constexpr std::chrono::seconds kReconnectDelay{1};

    void run(std::stop_token stop_token) {
        while (!stop_token.stop_requested()) {
            sp_port* serial_port = nullptr;
            if (sp_get_port_by_name(device_path_.c_str(), &serial_port) != SP_OK) {
                std::this_thread::sleep_for(kReconnectDelay);
                continue;
            }
            if (sp_open(serial_port, SP_MODE_READ) != SP_OK) {
                sp_free_port(serial_port);
                std::this_thread::sleep_for(kReconnectDelay);
                continue;
            }

            is_connected_.store(true);
            read_until_disconnected(serial_port, stop_token);
            is_connected_.store(false);

            sp_close(serial_port);
            sp_free_port(serial_port);
        }
    }

    void read_until_disconnected(sp_port* serial_port, std::stop_token stop_token) {
        std::string pending_bytes;
        std::array<char, 256> chunk{};

        while (!stop_token.stop_requested()) {
            const int byte_count =
                sp_blocking_read(serial_port, chunk.data(), chunk.size(), kReadTimeoutMilliseconds);
            if (byte_count < 0) {
                return; // port error or cable unplugged
            }
            if (byte_count == 0) {
                continue; // read timeout
            }
            pending_bytes.append(chunk.data(), static_cast<std::size_t>(byte_count));
            consume_complete_lines(pending_bytes);
        }
    }

    void consume_complete_lines(std::string& pending_bytes) {
        while (true) {
            const std::size_t newline_position = pending_bytes.find('\n');
            if (newline_position == std::string::npos) {
                break;
            }

            std::string_view line{pending_bytes.data(), newline_position};
            if (!line.empty() && line.back() == '\r') {
                line.remove_suffix(1); // Arduino sends "\r\n"
            }
            if (const auto status = parse_telemetry_line(line)) {
                publish(*status);
            }
            pending_bytes.erase(0, newline_position + 1);
        }

        if (pending_bytes.size() > kMaximumPendingBytes) {
            pending_bytes.clear(); // resync after a garbage burst
        }
    }

    void publish(SystemStatus status) {
        // The Arduino sends no timestamp, so stamp the arrival time here.
        status.time_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        const std::scoped_lock lock{telemetry_mutex_};
        latest_telemetry_ = status;
    }

    std::string device_path_;
    mutable std::mutex telemetry_mutex_;
    std::optional<SystemStatus> latest_telemetry_;
    std::atomic<bool> is_connected_{false};
    std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
    std::jthread reader_; // declared last: joins before the members above are destroyed
};

class StatusController {
  public:
    explicit StatusController(
        std::string serial_device_path = std::string{kDefaultSerialDevicePath})
        : telemetry_source_{std::move(serial_device_path)},
          demo_start_time_{std::chrono::steady_clock::now()} {
        current_status_ = get_demo_sensor_readings();
    }

    void update_status() {
        current_status_ =
            is_demo_data_enabled_ ? get_demo_sensor_readings() : get_live_sensor_readings();
    }

    void set_demo_data_enabled(bool is_enabled) { is_demo_data_enabled_ = is_enabled; }

    [[nodiscard]] bool is_demo_data_enabled() const { return is_demo_data_enabled_; }
    [[nodiscard]] bool is_telemetry_connected() const { return telemetry_source_.is_connected(); }
    [[nodiscard]] const SystemStatus& current_status() const { return current_status_; }

  private:
    // The cart slides back and forth along the rail.
    static constexpr double kCartMaximumOffsetMeters = 0.35;
    static constexpr double kCartMotionPeriodSeconds = 5.2;

    // The rod sways around upright.
    static constexpr double kRodMaximumTiltRadians = 1.0;
    static constexpr double kRodMotionPeriodSeconds = 3.0;
    static constexpr double kRodMotionPhaseOffsetRadians = 2.0;

    static constexpr double kTwoPiRadians = 2.0 * std::numbers::pi;

    SerialTelemetrySource telemetry_source_;
    bool is_demo_data_enabled_{false};
    std::chrono::steady_clock::time_point demo_start_time_{};
    SystemStatus current_status_{};

    [[nodiscard]] SystemStatus get_live_sensor_readings() const {
        return telemetry_source_.latest_telemetry().value_or(current_status_);
    }

    [[nodiscard]] SystemStatus get_demo_sensor_readings() const {
        const double elapsed_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - demo_start_time_)
                .count();

        const double cart_phase_radians =
            kTwoPiRadians * elapsed_seconds / kCartMotionPeriodSeconds;
        const double rod_phase_radians = kTwoPiRadians * elapsed_seconds / kRodMotionPeriodSeconds +
                                         kRodMotionPhaseOffsetRadians;

        const double cart_phase_rate_radians_per_second = kTwoPiRadians / kCartMotionPeriodSeconds;
        const double rod_phase_rate_radians_per_second = kTwoPiRadians / kRodMotionPeriodSeconds;

        SystemStatus status{};
        status.time_seconds = elapsed_seconds;
        status.cart_position_meters = kCartMaximumOffsetMeters * std::sin(cart_phase_radians);
        status.cart_velocity_meters_per_second = kCartMaximumOffsetMeters *
                                                 cart_phase_rate_radians_per_second *
                                                 std::cos(cart_phase_radians);
        status.cart_acceleration_meters_per_second_squared =
            -kCartMaximumOffsetMeters * cart_phase_rate_radians_per_second *
            cart_phase_rate_radians_per_second * std::sin(cart_phase_radians);
        status.rod_angle_radians = kRodMaximumTiltRadians * std::sin(rod_phase_radians);
        status.rod_angular_velocity_radians_per_second = kRodMaximumTiltRadians *
                                                         rod_phase_rate_radians_per_second *
                                                         std::cos(rod_phase_radians);
        return status;
    }
};
} // namespace pendulum::gui
