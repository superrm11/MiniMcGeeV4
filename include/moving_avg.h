#pragma once

class MovingAverage
{
    public:
    MovingAverage(int itr);
    double update(double val);

    int itr;
    double *vals;
    double out;
};