#ifndef _PID_H_
#define _PID_H_

#include <cmath>

void PID( double current, double target, double & output, double & err_last, double & err_total, 
    double Kp = 0.08025, double Ki = 0.06015, double Kd = 0.02725, double LOW_BOUND = -0.15, double HIGH_BOUND = 0.15);

#endif