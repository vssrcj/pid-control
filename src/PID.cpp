#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

enum Type { P, PI, _PID, TWIDDLE };

static Type type = TWIDDLE;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  // p_ = 0;
  // dp_ = 1;
  i_ = 0;
  steps = 0;
  // std::cout << "SET: " << "i_: " << i_ << std::endl;
  dp[0] = 0.2;
  dp[1] = 3.0;
  dp[2] = 0.004;
  best_err_ = -1;
  twiddle_state_ = 0;
  // Previous cte.
  // prev_cte = 0.0;

  // // Counters.
  // counter = 0;
  // errorSum = 0.0;
  // minError = std::numeric_limits<double>::max();
  // maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  // if (steps < 20) {
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
  // }
  // std::cout << "i_: " << i_ << ", Kp_: " << Kp_ << ", Kd_: " << Kd_ << ", Ki_: " << Ki_ << std::endl;
  std::cout << "d0: " << dp[0] << ", d1: " << dp[1] << ", d2: " << dp[2] << std::endl;
  std::cout << "d_error_: " << d_error_ << ", p_error_: " << p_error_ << ", i_error_: " << i_error_ << std::endl;
  // d_error_ = dp[0];
  // p_error_ = dp[1];
  // i_error_ = dp[2];

  if (best_err_ < 0) {
    best_err_ = cte;
    return;
  }

  double sum = dp[0] + dp[1] + dp[2];

  if (sum < 0.2) {
    return;
  }
  steps += 1;
  double p[] = { Kp_, Kd_, Ki_ };
  i_ = i_ > 2 ? 0 : i_;
  if (twiddle_state_ == 0) {
    p[i_] += dp[i_];
    // std::cout << p[i_] << std::endl;
  }
 
  if (cte < best_err_) {
    best_err_ = cte;
    dp[i_] *= 1.1;
    i_ += 1;
    twiddle_state_ = 0;
    // std::cout << "1: " << dp[i_] << std::endl;
  } else {
    if (twiddle_state_ == 0) {
      p[i_] -= 2 * dp[i_];
      // std::cout << "1: " << p[i_] << std::endl;
      twiddle_state_ = 1;
    } else {
      p[i_] += dp[i_];
      dp[i_] *= 0.9;
      // std::cout << "3: " << p[i_] << std::endl;
      // std::cout << "4: " << dp[i_] << std::endl;
      twiddle_state_ = 0;
      i_ += 1;
    }
  }
  Kp_ = p[0];
  Kd_ = p[1];
  Ki_ = p[2];
  // if (steps >= 20) {
  // d_error_ = dp[0];
  // p_error_ = dp[1];
  // i_error_ = dp[2];
  // }
  std::cout << "i_: " << i_ << ", Kp_: " << Kp_ << ", Kd_: " << Kd_ << ", Ki_: " << Ki_ << std::endl << std::endl;
  // std::cout << "i_: " << i_ << ", Kp_: " << Kp_ << ", Kd_: " << Kd_ << ", Ki_: " << Ki_ << std::endl << std::endl;
  // p_error_ = cte;
  // i_error_ += cte;
  // d_error_ = cte - p_error_;
}

double PID::TotalError() {
  if (type == P) {
    return 0.05 * p_error_;
  }
  if (type == PI) {
    return (0.2 * p_error_) + (3.0 * d_error_);
  }
  if (type == _PID) {
    return (0.2 * p_error_) + (3.0 * d_error_) + (0.004 * i_error_);
  }
  if (type == TWIDDLE) {
    // double norm = 1 / (Kp_ + Kd_ + Ki_);
    double result = 0.01 * ((Kp_ * p_error_) + (Kd_ * d_error_) + (Ki_ * i_error_));
    if (result < -1) return -1.0;
    if (result > 1) return 1.0;
    return result;
    // return (Kp_ * p_error_) +
    // (Kd_ * d_error_) +
    // (Ki_ * i_error_);
  }
  return 0.0;
  // return (Kp_ * p_error_) + (Kd_ * d_error_) + (Ki_ * i_error_);
}

