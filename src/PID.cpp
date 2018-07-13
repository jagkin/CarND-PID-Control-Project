#include "PID.h"
#include <iostream>
#include <limits>
#include <cmath>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

/*
 * Constructor.
 */
PID::PID() {
  // Initialize errors to 0.
  p_error = 0;
  i_error = 0;
  d_error = 0;

  // Initialize coefficients with 0
  K.push_back(0);  // Kp
  K.push_back(0);  // Ki
  K.push_back(0);  // Kd

  // Optimizer stuff
  twid_state = NOT_STARTED;

  dK.push_back(0.0);  // dKp
  dK.push_back(0.0);  // dKi
  dK.push_back(0.0);  // dKd
  thresh = 0.0;

  n_twiddles = 0;
  cur_error_sum = 0;
  best_error = numeric_limits<double>::max();
  n_runs_since_twiddle = 0;
  index = 0;  // Start twiddle with first coeff
}

/*
 * Destructor.
 */
PID::~PID() {
  // Nothing to do.
}

/*
 * Initializes the PID controller.
 * param [in] Kp: Initial Proportional gain.
 * param [in] Ki: Initial Integral gain.
 * param [in] Kd: Initial Differential gain.
 *
 */
void PID::Init(double Kp, double Ki, double Kd) {
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;

  // Set dKs to be 20% initial coefficients.
  dK[0] = Kp * 0.2;
  dK[1] = Ki * 0.2;
  dK[2] = Kd * 0.2;

  // Set threshold to 5% of the initial sum.
  thresh = (dK[0] + dK[1] + dK[2]) * 0.05;
}

/*
 * Updates error.
 * param [in] cross track error.
 */
void PID::UpdateError(double cte) {
  d_error = (cte - p_error);  // p_error holds prev_cte
  p_error = cte;
  i_error += cte;

  // Call the optimizer.
  Optimizer(cte);
}

/*
 * Return total error. Computed as
 * Kp * p_error + Ki * i_error + Kd * d_error;
 */
double PID::TotalError() {
  return K[0] * p_error + K[1] * i_error + K[2] * d_error;
}

void PID::LogTwiddle(void) {
  string state("INVALID");
  if (twid_state == ADD_TRIED) {
    state = "ADD";
  } else if (twid_state == SUB_TRIED) {
    state = "SUB";
  }
  cout << "Twiddle: " << n_twiddles << " state: " << state;
  cout << "Kp:" << K[0] << " Ki:" << K[1] << " Kd:" << K[2] << endl;
  cout << "dKp:" << dK[0] << " dKi:" << dK[1] << " dKd:" << dK[2] << endl;
}

/*
 * Gain optimizer.
 * param [in] cte. Current cross track error.
 */
void PID::Optimizer(double cte) {
  if ((twid_state == ALL_DONE) || (twid_state == FAILED)) {
    // Nothing to do.
    return;
  }

  // Sum absolute error over N_RUNS_PER_TWIDDLE runs.
  cur_error_sum += fabs(cte);
  if (n_runs_since_twiddle++ < N_RUNS_PER_TWIDDLE) {
    // Nothing to do yet.
    return;
  }

  // Average the error.
  double cur_error = cur_error_sum / N_RUNS_PER_TWIDDLE;

  // Go twiddle
  n_twiddles++;
  // Reset cur_error and counter.
  cur_error_sum = 0;
  n_runs_since_twiddle = 0;

  // Check if we just started.
  if (n_twiddles == 1) {
    // cur_error is the error with init params.
    best_error = cur_error;

    cout << "Optimizer: Twiddling start\n cur_error:" << cur_error << endl;

    // Very first twiddle.
    // Try ADD
    K[index] += dK[index];
    twid_state = ADD_TRIED;
    LogTwiddle();
    return;
  }

  // Check if there was improvement since last twiddle.
  bool error_improved = cur_error < best_error;

  // Flag to indicate if it is time to twiddle next coefficient.
  bool twiddle_next_coeff(false);

  switch (twid_state) {
    case ADD_TRIED:
      if (error_improved) {
        // ADD helped already.
        // Bump up dK and move on to next coeff
        dK[index] *= 1.10;
        twiddle_next_coeff = true;
      } else {
        // ADD did not help. Try SUB.
        K[index] -= 2 * dK[index];  // dK[index] was added previously.
        twid_state = SUB_TRIED;
      }
      break;
    case SUB_TRIED:
      if (error_improved) {
        // SUB helped.
        // Bump up dK.
        dK[index] *= 1.10;
      } else {
        // Both ADD and SUB did not help.
        // Reset value back to original. Bump down dK
        K[index] += dK[index];
        dK[index] *= 0.90;  // Decrease by 10%
      }
      // Both ADD and SUB tried move on to next coeff
      twiddle_next_coeff = true;
      break;
    default:
      // Control should never reach here.
      cout << "Something went wrong in twiddling\n";
      twid_state = FAILED;
      break;
  }

  // Update best error and bestK
  if (error_improved) {
    best_error = cur_error;
    bestK = K;
    cout << "Error improved: best_error:" << best_error << endl;
  }

  if (twiddle_next_coeff) {
    // Increment index to try next co-efficient
    if (index == (dK.size() - 1))
      index = 0;
    else
      index++;

    // Try ADD.
    K[index] += dK[index];
    twid_state = ADD_TRIED;
  }

  // Check if we reached the threshold.
  double sum_dk = dK[0] + dK[1] + dK[2];
  if (sum_dk < thresh) {
    cout << "Reached threshold. sum_dK:" << sum_dk << " thresh:" << thresh
         << endl;
    cout << "Best coefficients\n";
    cout << "Kp:" << bestK[0] << " Ki:" << bestK[1] << " Kd:" << bestK[2]
         << endl;
    twid_state = ALL_DONE;
    return;
  }

  LogTwiddle();
  return;
}
