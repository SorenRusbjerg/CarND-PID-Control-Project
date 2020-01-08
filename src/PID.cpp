#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  TwiddleState = Twiddle_Init;
  parNum = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  K_pid[0] = Kp_;
  K_pid[1] = Ki_;
  K_pid[2] = Kd_;

  cte_last = 0.0;

  Ts = 1/25.0;  // best guess
  best_err = 1.0e6;

  dp_pid[0] = 0.05;   // KP
  dp_pid[1] = 0.015;  // KI
  dp_pid[2] = 0.0039; // KD
  twiddle_err = 0.0;

}


void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */

  // PID control
  p_error = cte * K_pid[0];
  i_error += cte * K_pid[1] * Ts;  

  // Wind-up saturation of i-term
  double sat_val = 0.5;
  if (i_error > sat_val)
  {
    i_error = sat_val;
  }
  else if (i_error < -sat_val)
  {
    i_error = -sat_val;
  }

  d_error = K_pid[2] * (cte - cte_last)/Ts;
  cte_last = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double error = p_error + i_error + d_error; 

  // Limit error jused for steering to interval [-1; 1]
  if (error > 1.0)
  {
    error = 1.0;
  }
  else if (error < -1.0)
  {
    error = -1.0;
  }

  return p_error + i_error + d_error;  // TODO: Add your total error calc here!
}



// Update twidddle when new error_sum is ready
void PID::UpdateTwiddle(double error_sum)
{
  if (TwiddleState == Twiddle_Init)
  {    
    K_pid[parNum] += dp_pid[parNum];   
    TwiddleState = Test_Positive;
  }
  else if (TwiddleState == Test_Positive)
  {
    if (error_sum < best_err)
    {
      best_err = error_sum;
      dp_pid[parNum] *= 1.1;

      // Start new twiddle
      TwiddleState = Test_Positive;
      parNum++;
      parNum %= 3;      
      K_pid[parNum] += dp_pid[parNum];   
      TwiddleState = Test_Positive;
    }
    else
    {
      K_pid[parNum] -= 2.0*dp_pid[parNum];
      TwiddleState = Test_Negative;
    }
  }
  else if (TwiddleState == Test_Negative)
  {
    if (error_sum < best_err)
    {
      best_err = error_sum;
      dp_pid[parNum] *= 1.1;      
    }
    else
    {
      K_pid[parNum] += dp_pid[parNum];
      dp_pid[parNum] *= 0.9;
    }
      // Start new twiddle
      TwiddleState = Test_Positive;
      parNum++;
      parNum %= 3;      
      K_pid[parNum] += dp_pid[parNum];   
      TwiddleState = Test_Positive;
  }
}
/* Twiddle Python
best_err = 1e6
while sum(dp) > tol:
    for n in range(len(p)):
        p[n]+=dp[n]
        err = run(robot, p)
        if err < best_err:
            best_err = err
            dp[n]*=1.1
        else:
            p[n]-=2*dp[n]
              err = run(robot, p)
            if err < best_err:
                best_err = err
                dp[n]*=1.1
            else:
                p[n]+=dp[n]
                dp[n]*=0.9
*/
