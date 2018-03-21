#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp) {
  PID::Init(Kp, -1, -1);
  
  param_count = 1;
}

void PID::Init(double Kp, double Kd) {
  PID::Init(Kp, Kd, -1);
  
  param_count = 2;
}

void PID::Init(double Kp, double Kd, double Ki) {
  param_count = 3;
  
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  i = 0;
  steps = 0;
  dp[0] = Kp;
  dp[1] = Kd;
  dp[2] = Ki;
  best_err = -1;
  twiddle_repeat = false;
  use_twiddle = false;
}

void PID::ActivateTwiddle() {
  use_twiddle = true;
  // Begin all coefficients at 0.
  Kp = 0;
  Ki = 0;
  Kd = 0;
}

void PID::UpdateError(double cte) {
  steps += 1;

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (!use_twiddle) return;
 
  if (best_err < 0) {
    // If first run.
    best_err = cte;
    return;
  }

  double sum = dp[0] + dp[1] + dp[2];

  if (sum < 0.2) {
    // Done optimizing.
    return;
  }

  i %= param_count;

  double p[] = { Kp, Kd, Ki };

  if (!twiddle_repeat) {
    p[i] += dp[i];
  }
 
  if (cte < best_err) {
    best_err = cte;
    dp[i] *= 1.1;
    i += 1;
    twiddle_repeat = false;
  } else {
    if (!twiddle_repeat) {
      p[i] -= 2 * dp[i];
      twiddle_repeat = true;
    } else {
      p[i] += dp[i];
      dp[i] *= 0.9;
      twiddle_repeat = false;
      i += 1;
    }
  }
  Kp = p[0];
  Kd = p[1];
  Ki = p[2];
}

double PID::TotalError() {
  double result;
  if (param_count == 1) {
    result = (Kp * p_error);
  } else if (param_count == 2) {
    result = (Kp * p_error) + (Kd * d_error);
  } else {
    result = (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
  }
  if (use_twiddle) {
    // Smoothing element.
    result *= 0.02;
  }
  return result > 1 ? 1 : (result < -1 ? 1 : result);
}
