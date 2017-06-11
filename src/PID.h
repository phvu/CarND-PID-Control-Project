#ifndef PID_H
#define PID_H

#include <string.h>
#include "twiddle.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  bool twiddle_enabled;
  Twiddle<3> twiddle;
  int step;
  double twiddle_error;

  /*
  * Constructor
  */
  PID(bool enable_twiddle, std::string controller_name);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(int twiddle_every, double Kp, double Ki, double Kd, double dKp, double dKi, double dKd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /**
   * Reset the controller
   */
  void reset();
};

#endif /* PID_H */
