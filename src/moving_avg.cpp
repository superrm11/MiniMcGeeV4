#include "moving_avg.h"
#include <cstdlib>

MovingAverage::MovingAverage(int itr) : itr(itr)
{
    vals = (double*) malloc(itr * sizeof(double));
}

double MovingAverage::update(double val)
{
    for (int i = 0; i < itr-1; i++)
    {
        vals[i] = vals[i+1];
    }

    vals[itr-1] = val;

    double sum = 0;
    for(int i = 0; i < itr; i++)
    {
        sum += vals[i];
    }

    return out = sum / itr;
}
