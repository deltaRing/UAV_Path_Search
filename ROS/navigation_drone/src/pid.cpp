#include "pid.h"

void PID( double current, double target, double & output, double & err_last, double & err_total,
    double Kp, double Ki, double Kd, double LOW_BOUND, double HIGH_BOUND){
    double err = target - current;
    err_total += err;
    output = Kp * err + Ki * err_total + Kd * (err - err_last);
    err_last = err;
    if (output > HIGH_BOUND) output = HIGH_BOUND;
    else if (output < LOW_BOUND) output = LOW_BOUND;
}
