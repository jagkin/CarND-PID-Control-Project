#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /*
   * Errors.
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients. Kp, Ki and Kd
   */
  std::vector<double> K;
  int n_updates;

  // Twiddle stuff
  enum twiddle_state {
    NOT_STARTED = 0,
    STARTED,
    ADD_TRIED,
    SUB_TRIED,
    ALL_DONE
  };

  bool twiddle;
  twiddle_state twid_state;
  std::vector<double> dK;
  double best_error;
  double thresh;
  int num_updates;
  double n_runs_since_twiddle;
  double cur_error;
  int n_twiddles;
  int index;

  static const int N_HISTORY_RUNS = 1410;

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
   * Twiddle co-efficients.
   */
  void Twiddle(double cur_error);

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
