#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  double dp[3];
  double best_err;
  bool twiddle_repeat;
  int i;
  int steps;
  bool use_twiddle;
  int param_count;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Active Twiddle.
  */
  void ActivateTwiddle();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);
  void Init(double Kp, double Ki);
  void Init(double Kp);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
