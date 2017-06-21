#ifndef PID_H
#define PID_H

class PID {
    
    double tol;
    double total_error;
    double best_error;
    double p[3];
    double dp[3];
    bool incremented;
    bool decremented;
    int steps_stabilization;
    int twiddle_period;
    int step;
    int param_index;
    
    void Twiddle();
    
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
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

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
