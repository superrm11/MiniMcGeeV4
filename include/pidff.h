#pragma once
#include <chrono>

using namespace std::chrono;

class PIDFF
{
    public:
    PIDFF();

    double update(double sensor);
    double kS, kV, kP, kI, kD;
    double target, out;
    double error, accum_error;

    time_point<system_clock> last_time;
    
};