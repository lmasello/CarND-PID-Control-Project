#include "PID.h"
#include <math.h>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;
}

void PID::UpdateError(double cte) {
  const double TIME = 1.0;
  d_error = (cte - p_error) / TIME;  // Difference in cte
  p_error = cte;
  i_error += cte;  // Summation of cte values
}

double PID::TotalError() {
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  if (steer < -1)
    return -1;
  else if (steer > 1)
    return 1;
  else
    return steer;
}


double PID::ThrottleValue() {
  static const double THROTTLE_HIGH_ERROR = 0.1;
  static const double HIGH_ANGLE = 10;  // Half of the default value
  static const double MAX_SPEED = 90.0;
  double throttle;

  if (fabs(TotalError()) > HIGH_ANGLE)
    throttle = THROTTLE_HIGH_ERROR;
  else  // Use the inverse of the steering value capped to 90
    throttle = fmin(1/fabs(TotalError()), MAX_SPEED);

  // Return the value in the range [0, 0.8]
  return throttle / MAX_SPEED;
}
