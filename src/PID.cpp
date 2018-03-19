#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

enum Type { P, PI, _PID };

static Type type = P;
static bool use_twiddle = false;

static int param_count = type == P ? 1 : (type == PI ? 2 : 3);

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  i = 0;
  steps = 0;
  dp[0] = Kp;
  dp[1] = Ki;
  dp[2] = Kd;
  p[0] = 0.0;
  p[1] = 0.0;
  p[2] = 0.0;
  best_err = -1;
  twiddle_repeat = false;
}

void PID::UpdateError(double cte) {
  steps += 1;

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (best_err < 0) {
    // If first run.
    best_err = cte;
    return;
  }

  double sum = dp[0] + dp[1] + dp[2];

  if (sum < 0.2) {
    return;
  }

  i %= param_count;

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
  if (type == P) {
    result = Kp * p_error;
  } else if (type == PI) {
    result = (Kp * p_error) + (Kd * d_error);
  } else {
    result = (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
  }
  if (use_twiddle) {
    result *= 0.01;
  }
  return result > 1 ? 1 : (result < -1 ? 1 : result);
}

