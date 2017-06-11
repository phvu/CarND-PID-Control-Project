#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <iostream>
#include <string.h>

/**
 * Implement twiddle for N-dimensional parameters
 * @tparam N number of dimensions
 */
template<int N>
class Twiddle {
  double p[N];
  double dp[N];
  double tolerance;

  int twiddle_every;

  std::string name;

  // boring flags
  int twidding_var;
  double best_err;
  int twiddling_step;
  bool initialized;

  double total_dp() {
    double s = 0;
    for (double v: dp) {
      s += v;
    }
    return s;
  }

  void print_debug() {
    std::cout << name << ": p and dp: ";
    for (int i = 0; i < N; ++i) {
      std::cout << p[i] << " ";
    }
    std::cout << "; ";
    for (int i = 0; i < N; ++i) {
      std::cout << dp[i] << " ";
    }
    std::cout << "; sum dp = " << total_dp() << "; best error " << best_err << std::endl;
  }

public:
  Twiddle(std::string name_) : name(name_) {}

  ~Twiddle() {}

  void init(int twiddle_steps, double tol, double initial_p[], double initial_dp[]) {
    twiddle_every = twiddle_steps;
    tolerance = tol;
    for (int i = 0; i < N; ++i) {
      p[i] = initial_p[i];
      dp[i] = initial_dp[i];
    }
    twidding_var = 0;
    initialized = false;
  }

  /**
   * Get the value in the ith dimension
   */
  double get(int i) {
    return p[i];
  }

  bool jiggle(int current_step, double err) {
    //  && (!initialized || err < best_err)
    if (current_step != twiddle_every) {
      return false;
    }

    if (total_dp() < tolerance) {
      //std::cout << "Converged. No twiddle needed" << std::endl;
      return true;
    }

    if (!initialized) {
      best_err = err;
      twidding_var = 0;
      p[twidding_var] += dp[twidding_var];
      twiddling_step = 1;
      initialized = true;
    }

    if (err < best_err) {
      // it works (whatever it is up or down), so move to the next variable
      best_err = err;
      dp[twidding_var] *= 1.05;

      twidding_var = (twidding_var + 1) % N;
      p[twidding_var] += dp[twidding_var];
      twiddling_step = 1;
    } else {

      if (twiddling_step == 1) {
        // we are twiddling it up, and it doesn't work
        // so tuning it down
        p[twidding_var] -= 2 * dp[twidding_var];
        twiddling_step = 2;
      } else {
        // we are twiddling it down, and it doesn't work
        // so reset it, decrease the twiddling rate, and move to the next variable
        p[twidding_var] += dp[twidding_var];
        dp[twidding_var] *= 0.95;

        twidding_var = (twidding_var + 1) % N;
        p[twidding_var] += dp[twidding_var];
        twiddling_step = 1;
      }

    }
    print_debug();
    return true;
  }
};

#endif /* TWIDDLE_H */