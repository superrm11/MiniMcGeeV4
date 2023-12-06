#include "pidff.h"

PIDFF::PIDFF(){};

double PIDFF::update(double sensor)
{
    // Get time delta (micros)
    auto cur_time(system_clock::now());
    double delta_sec = duration<double>(cur_time - last_time).count();
    last_time = cur_time;

    double old_error = error;
    error = target - sensor;
    accum_error += error * delta_sec;
    double deriv_error = (error - old_error) / delta_sec;

    // PID loop
    double p = kP * error;
    double i = kI * accum_error;
    double d = kD * deriv_error;

    // Feedforward loop
    double s = kS * ((error > 0) ? 1 : (error < 0) ? -1 : 0);
    double v = kV * target;

    return out = p + i + d + s + v;

}