#include "pid.h"

void PID(double & current, double & err_last, double & err_total, double target, double Kp, double Ki, double Kd){
    double err = target - current;
    err_total += err;
    current = Kp * err + Ki * err_total + Kd * (err - err_last);
    err_last = err;
}