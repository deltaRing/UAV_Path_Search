#ifndef _PID_H_
#define _PID_H_

#include <cmath>

void PID(double & current,  double & err_last, double & err_total, double target, double Kp = 0.5, double Ki = 1.0, double Kd = 1.0);

#endif