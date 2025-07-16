#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm> // For std::clamp
#include <cmath>     // For fabs
#include <iostream>

// PID Controller Class
class PID {
public:

    PID() 
        : Kp(0.0), Ki(0.0), Kd(0.0),
        output_min(0.0), output_max(0.0),
        integral(0.0), prev_error(0.0) {}

    /**
     * @brief Constructor for PID.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param output_min Minimum output limit
     * @param output_max Maximum output limit
     */
    PID(double kp, double ki, double kd, double output_min, double output_max, double anti_windup_gain);

    /**
     * @brief Updates the PID controller.
     * @param desired_value Desired value
     * @param current_value Current value
     * @param dt Time step
     * @return Control signal
     */
    double update(double desired_value, double current_value, double dt);

    void resetIntegral();

private:
    double Kp, Ki, Kd;        // PID coefficients
    double output_min, output_max; // Output clamping range
    double integral;          // Accumulated integral error
    double prev_error;        // Previous error for derivative calculation
    double anti_windup_gain;
};

#endif // PID_CONTROLLER_HPP
