#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void UpdateTwiddle(double cte);

  enum ETwiddleState {Twiddle_Init, Test_Positive, Test_Negative} TwiddleState; 

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double K_pid[3];
  double nom_speed;
  double Ts;
  double cte_last;

  /**
   * Twiddle Coefficients
   */   
  int parNum;
  double best_err;
  double dp_pid[3];
  double twiddle_err;
};

#endif  // PID_H