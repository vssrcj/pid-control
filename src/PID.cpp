#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

static float tau = 0.1; // 0.05 - 0.1

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  // Previous cte.
  // prev_cte = 0.0;

  // // Counters.
  // counter = 0;
  // errorSum = 0.0;
  // minError = std::numeric_limits<double>::max();
  // maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  p_error_ = tau * cte;
  // p_error_ = cte;
  // i_error_ += cte;
  // d_error_ = cte - p_error_;
}

double PID::TotalError() {
  return p_error_;
  // return (Kp_ * p_error_) + (Kd_ * d_error_) + (Ki_ * i_error_);
}

