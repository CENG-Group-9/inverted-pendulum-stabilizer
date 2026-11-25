#ifndef ORBIT_CAMERA_H
#define ORBIT_CAMERA_H

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace pendulum::gui {

class OrbitCamera {
  public:
    explicit OrbitCamera(glm::vec3 target_meters = glm::vec3(0.0F),
                         float distance_meters = kDefaultDistanceMeters,
                         float yaw_radians = kDefaultYawRadians,
                         float pitch_radians = kDefaultPitchRadians)
        : target_meters_(target_meters),
          distance_meters_(std::clamp(distance_meters, kMinDistanceMeters, kMaxDistanceMeters)),
          yaw_radians_(yaw_radians),
          pitch_radians_(std::clamp(pitch_radians, -kMaxPitchRadians, kMaxPitchRadians)) {}

    [[nodiscard]] glm::mat4 view_matrix() const {
        return glm::lookAt(position_meters(), target_meters_, kWorldUp);
    }

    [[nodiscard]] glm::mat4 projection_matrix(float aspect_ratio) const {
        const float safe_aspect_ratio = std::max(aspect_ratio, kMinAspectRatio);
        return glm::perspective(glm::radians(field_of_view_degrees_), safe_aspect_ratio,
                                kNearPlaneMeters, kFarPlaneMeters);
    }

    [[nodiscard]] glm::vec3 position_meters() const {
        const float horizontal_distance_meters = distance_meters_ * std::cos(pitch_radians_);

        return target_meters_ + glm::vec3(horizontal_distance_meters * std::cos(yaw_radians_),
                                          distance_meters_ * std::sin(pitch_radians_),
                                          horizontal_distance_meters * std::sin(yaw_radians_));
    }

    [[nodiscard]] glm::vec3 target_meters() const { return target_meters_; }
    [[nodiscard]] float distance_meters() const { return distance_meters_; }
    [[nodiscard]] float yaw_radians() const { return yaw_radians_; }
    [[nodiscard]] float pitch_radians() const { return pitch_radians_; }
    [[nodiscard]] float field_of_view_degrees() const { return field_of_view_degrees_; }

    void orbit(float delta_x_pixels, float delta_y_pixels) {
        yaw_radians_ += delta_x_pixels * kOrbitSensitivityRadiansPerPixel;
        yaw_radians_ = std::remainder(yaw_radians_, kTwoPiRadians);

        pitch_radians_ -= delta_y_pixels * kOrbitSensitivityRadiansPerPixel;
        pitch_radians_ = std::clamp(pitch_radians_, -kMaxPitchRadians, kMaxPitchRadians);
    }

    void pan(float delta_x_pixels, float delta_y_pixels) {
        const glm::vec3 forward = glm::normalize(target_meters_ - position_meters());
        const glm::vec3 right = glm::normalize(glm::cross(forward, kWorldUp));
        const glm::vec3 up = glm::normalize(glm::cross(right, forward));
        const float pan_meters_per_pixel = distance_meters_ * kPanSensitivityPerPixel;

        target_meters_ += right * (-delta_x_pixels * pan_meters_per_pixel);
        target_meters_ += up * (delta_y_pixels * pan_meters_per_pixel);
    }

    void zoom(float scroll_delta) {
        const float zoom_factor = std::pow(kZoomFactorPerScrollStep, scroll_delta);
        distance_meters_ =
            std::clamp(distance_meters_ * zoom_factor, kMinDistanceMeters, kMaxDistanceMeters);
    }

  private:
    static constexpr float kPiRadians = 3.14159265358979323846F;
    static constexpr float kTwoPiRadians = 2.0F * kPiRadians;
    static constexpr float kDegreesToRadians = kPiRadians / 180.0F;

    static constexpr float kDefaultDistanceMeters = 3.4F;
    static constexpr float kMinDistanceMeters = 0.5F;
    static constexpr float kMaxDistanceMeters = 20.0F;
    static constexpr float kDefaultYawRadians = 61.0F * kDegreesToRadians;
    static constexpr float kDefaultPitchRadians = 21.0F * kDegreesToRadians;

    static constexpr float kMaxPitchRadians = 89.0F * kDegreesToRadians;
    static constexpr float kMinAspectRatio = 0.01F;
    static constexpr float kNearPlaneMeters = 0.05F;
    static constexpr float kFarPlaneMeters = 100.0F;
    static constexpr float kOrbitSensitivityRadiansPerPixel = 0.005F;
    static constexpr float kPanSensitivityPerPixel = 0.002F;
    static constexpr float kZoomFactorPerScrollStep = 0.9F;
    static constexpr float kDefaultFieldOfViewDegrees = 45.0F;

    inline static const glm::vec3 kWorldUp{0.0F, 1.0F, 0.0F};

    glm::vec3 target_meters_{};
    float distance_meters_{kDefaultDistanceMeters};
    float yaw_radians_{kDefaultYawRadians};
    float pitch_radians_{kDefaultPitchRadians};
    float field_of_view_degrees_{kDefaultFieldOfViewDegrees};
};

} // namespace pendulum::gui

#endif
