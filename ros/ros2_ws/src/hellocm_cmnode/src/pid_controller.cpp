#include "pid_controller.hpp"

// Constructor
PID::PID(double kp, double ki, double kd, double output_min, double output_max, double anti_windup_gain)
    : Kp(kp), Ki(ki), Kd(kd),
      output_min(output_min), output_max(output_max),
      anti_windup_gain(anti_windup_gain),
      integral(0.0), prev_error(0.0) {}

// Update function
double PID::update(double desired_value, double current_value, double dt) {
    // Calculate error
    double error = desired_value - current_value;

    // Proportional term
    double P_out = Kp * error;

    // Integral term
    integral += error * dt;
    double I_out = Ki * integral;

    // Derivative term
    double derivative = (error - prev_error) / dt;
    double D_out = Kd * derivative;

    // Calculate total output
    double output = P_out + I_out + D_out;

    // Anti-windup: Back-calculation if output is saturated
    if (output > output_max) {
        output = output_max;
        integral -= anti_windup_gain * (output - output_max) * dt;
    } else if (output < output_min) {
        output = output_min;
        integral -= anti_windup_gain * (output - output_min) * dt;
    }

    // Clamp output
    output = std::clamp(output, output_min, output_max);

    // Update previous error
    prev_error = error;

    return output;
}

// Reset integral term
void PID::resetIntegral() {
    integral = 0.0;
    prev_error = 0.0;
}