#pragma once
#include <GLFW/glfw3.h>
#include <cmath>

namespace pendulum::gui {
    struct SystemStatus
    {
        public:
        double rodAngle; // 0 is upright in radians
        double rodAngularVelocity; // in rad/s
        double cartPosition; // between -0.5 and 0.5
        double cartVelocity; // in m/s
        double cartAcceleration; // TODO: optional not sure to hold
        double time;
    };

    class StatusController
    {
        bool isModuleDemo = true;

        public:
        SystemStatus currentStatus;
        StatusController()
        {
            currentStatus.rodAngle = 1.0;
            currentStatus.rodAngularVelocity = 0.0;
            currentStatus.cartPosition = 0.0;
            currentStatus.cartVelocity = 0.0;
            currentStatus.cartAcceleration = 0.0;
            currentStatus.time = glfwGetTime();
        }

        void update_status() 
        {
            if (isModuleDemo)
            {
                currentStatus = get_dummy_sensor_readings();
            }
            else
            {
                currentStatus = get_sensor_readings();
            }
        }

        SystemStatus get_sensor_readings()
        {
            SystemStatus status;
            return status;
        }

        SystemStatus get_dummy_sensor_readings()
        {   
            SystemStatus status;
            status.rodAngle = std::sin(glfwGetTime());
            status.rodAngularVelocity = std::cos(glfwGetTime());
            status.cartPosition = std::log(glfwGetTime());
            status.cartVelocity = std::exp(glfwGetTime());
            status.cartAcceleration = 10.0;
            status.time = glfwGetTime();
            return status;
        }
    };
}