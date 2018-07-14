#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
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
   * Initialize PID optimizer.
   */
  void InitOptimizer(double dKp, double dKi, double dKd, double threshold);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

 private:
  /*
   * Coefficients. Kp, Ki and Kd
   */
  std::vector<double> K;

  /*
   * Errors.
   */
  double p_error;
  double i_error;
  double d_error;

  // Twiddle state.
  enum twiddle_state {
    NOT_STARTED,
    ADD_TRIED,
    SUB_TRIED,
    ALL_DONE,
    FAILED
  };

  twiddle_state twid_state;
  std::vector<double> dK;
  double best_error;
  std::vector<double> bestK;
  double thresh;
  double n_runs_since_twiddle;
  double cur_error_sum;
  int n_twiddles;
  int index;

  // The UpdateError() gets called about 640 times for one lap.
  // Make sure we let algorithm to settle for one lap.
  static const int N_RUNS_TO_EVAL = 640;
  // Make sure we evaluate each twiddle over for atleast one lap.
  static const int N_RUNS_TO_SETTLE = 640;
  /*
   * Optimize co-efficients using twiddle.
   */
  void Optimizer(double cte);

  /*
   * Logs twiddle
   */
  void LogTwiddle(void);
};

#endif /* PID_H */
