#include "PID.h"
#include <math.h>

/*
* TODO: Complete the PID class.
*/

PID::PID(bool enable_twiddle, std::string controller_name)
: twiddle(controller_name) {
  twiddle_enabled = enable_twiddle;
  reset();
}

PID::~PID() {}

void PID::Init(int twiddle_every, double Kp, double Ki, double Kd, double dKp, double dKi, double dKd) {
  double arr[] = {Kp, Ki, Kd};
  double arr2[] = {dKp, dKi, dKd};
  twiddle.init(twiddle_every, 0.00001, arr, arr2);
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  step += 1;
  twiddle_error += (cte * cte);
  if (twiddle_enabled && twiddle.jiggle(step, twiddle_error / step)) {
    reset();
  }
}

double PID::TotalError() {
  return twiddle.get(0) * p_error + twiddle.get(1) * i_error + twiddle.get(2) * d_error;
}

void PID::reset() {
  p_error = i_error = d_error = 0.0;
  step = 0;
  twiddle_error = 0;
}
