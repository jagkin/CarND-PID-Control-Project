#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;

  // Initialize coefficients with 0
  K.push_back(0);  // Kp
  K.push_back(0);  // Ki
  K.push_back(0);  // Kd
  n_updates = 0;

  // Twidlle stuff
  twiddle = false;
  twid_state = NOT_STARTED;
  dK.push_back(0.01);  // dKp
  dK.push_back(0.002);  // dKi
  dK.push_back(0.08);  // dKd
  //Kp:0.129139 Ki:0.021 Kd:0.767874

  best_error = numeric_limits<double>::max();
  thresh = (dK[0] + dK[1] + dK[2])*0.05;
  num_updates = 0;
  n_runs_since_twiddle = 0;
  cur_error = 0;
  n_twiddles = 0;
  index = 0;
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
}

void PID::Twiddle(double cur_error) {
  // Check if we reached the threshold
  double sum_dk = dK[0] + dK[1] + dK[2];
  if (sum_dk < thresh) {
    cout << "Done twiddling.. sum_dk:" << sum_dk << endl;
    cout << "Final parameters";
    cout << " Kp:" << K[0] << " Ki:" << K[1] << " Kd:" << K[2] << endl;
    twid_state = ALL_DONE;
    return;
  }

  if (twid_state == STARTED) {
    // Very first twiddle.
    K[index] += dK[index];
    twid_state = ADD_TRIED;
    return;
  }

  bool twiddle_next_coeff = false;
  switch (twid_state) {
    case ADD_TRIED:
      if (cur_error < best_error) {
        // ADD helped
        best_error = cur_error;
        dK[index] *= 1.10;  // Increase by 10%
        twiddle_next_coeff = true;
      } else {
        // ADD did not help. Try subtract
        K[index] -= 2 * dK[index];
        twid_state = SUB_TRIED;
      }
      break;
    case SUB_TRIED:
      if (cur_error < best_error) {
        // SUB helped
        best_error = cur_error;
        dK[index] *= 1.10;  // Increase by 10%
      } else {
        // Both ADD and SUB did not help.
        K[index] += dK[index];  // Reset value back to original
        dK[index] *= 0.90;  // Decrease by 10%
      }
      twiddle_next_coeff = true;
      break;
    default:
      cout << "Something wrong.\n";
      break;
  }

  if (twiddle_next_coeff) {
    // Increment index to try next co-efficient
    if (index == (dK.size() - 1))
      index = 0;
    else
      index++;

    K[index] += dK[index];
    twid_state = ADD_TRIED;
  }

  return;
}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error);  // p_error holds prev_cte
  p_error = cte;
  i_error += (cte/N_HISTORY_RUNS);

  if (n_updates == N_HISTORY_RUNS) {
    // Ideally i_error should hold moving average of last N_HISTORY_RUNS cte.
    i_error = 0;
  }

  if (twid_state == ALL_DONE) {
    // Twiddle done. Nothing to do.
    return;
  }

  // Keep track of error for twiddles
  cur_error += (cte * cte);

  n_runs_since_twiddle++;
  if (n_runs_since_twiddle == N_HISTORY_RUNS) {
    if (twid_state == NOT_STARTED) {
      // Twiddling not started yet. Start now.
      best_error = cur_error;  // This is the error with init params.
      twid_state = STARTED;
    }
    twiddle = true;
  }

  if (twiddle == false) {
    // Nothing to do yet.
    return;
  }

  cout << "CTE:" << cte << " cur_error:" << cur_error << " best_error" << best_error << endl;
  if (cur_error < best_error) {
    cout << "Improvement observed after twiddle:" << n_twiddles << endl;
  }

  // Twiddle params
  Twiddle(cur_error);
  twiddle = false;
  n_twiddles++;
  cur_error = 0;
  n_runs_since_twiddle = 0;

  cout << "Parameters after twiddle:" << n_twiddles << endl;
  cout << " Kp:" << K[0] << " Ki:" << K[1] << " Kd:" << K[2] << endl;
  cout << " dKp:" << dK[0] << " dKp:" << dK[1] << " dKd:" << dK[2] << endl;
}

double PID::TotalError() {
  return K[0] * p_error + K[1] * i_error + K[2] * d_error;
}
