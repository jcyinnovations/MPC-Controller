#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double v_ref = 40;

size_t x_idx      = 0;
size_t y_idx      = x_idx   + N;
size_t psi_idx    = y_idx   + N;
size_t v_idx      = psi_idx + N;
size_t cte_idx    = v_idx   + N;
size_t epsi_idx   = cte_idx + N;
size_t delta_idx  = epsi_idx+ N;
size_t a_idx      = delta_idx + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    fg[0] = 0;
    /**
     * Cost function: state errors and target velocity
     * Reduce influence of CTE to 5% to avoid over-correction (oscillation)
     */
    for (int t = 0; t < N; t++) {
      fg[0] += 0.05*CppAD::pow(vars[cte_idx + t], 2);
      fg[0] += CppAD::pow(vars[epsi_idx + t], 2);
      fg[0] += CppAD::pow(vars[v_idx + t] - v_ref, 2);
    }

    /**
    * Cost function: minimize actuations
    **/
    for (int t = 0; t < N-1; t++) {
      fg[0] += CppAD::pow(vars[delta_idx + t], 2);
      fg[0] += CppAD::pow(vars[a_idx + t], 2);
    }

    /**
     * Minimize difference between sequential actuations
     **/
    for (int t = 0; t < N-2; t++) {
      fg[0] += CppAD::pow(vars[delta_idx + t + 1] - vars[delta_idx + t], 2);
      fg[0] += CppAD::pow(vars[a_idx + t + 1] - vars[a_idx + t], 2);
    }

    fg[x_idx   + 1] = vars[x_idx];
    fg[y_idx   + 1] = vars[y_idx];
    fg[psi_idx + 1] = vars[psi_idx];
    fg[v_idx   + 1] = vars[v_idx];
    fg[cte_idx + 1] = vars[cte_idx];
    fg[epsi_idx+ 1] = vars[epsi_idx];

    for (int t = 1; t < N; t++) {
      AD<double> x1   = vars[x_idx    + t];
      AD<double> y1   = vars[y_idx    + t];
      AD<double> psi1 = vars[psi_idx  + t];
      AD<double> v1   = vars[v_idx    + t];
      AD<double> cte1 = vars[cte_idx  + t];
      AD<double> epsi1= vars[epsi_idx + t];

      AD<double> x0   = vars[x_idx    + t - 1];
      AD<double> y0   = vars[y_idx    + t - 1];
      AD<double> psi0 = vars[psi_idx  + t - 1];
      AD<double> v0   = vars[v_idx    + t - 1];
      AD<double> cte0 = vars[cte_idx  + t - 1];
      AD<double> epsi0= vars[epsi_idx + t - 1];

      AD<double> delta0 = vars[delta_idx + t - 1];
      AD<double> a0 = vars[a_idx + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + t + x_idx]   = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + t + y_idx]   = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + t + psi_idx] = psi1  - (psi0 + (v0/Lf) * delta0 * dt);
      fg[1 + t + v_idx]   = v1    - (v0 + a0 * dt);
      fg[1 + t + cte_idx] = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + t + epsi_idx]= epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  int state_length = 6; //state.rows();
  int actuators = 2;
  size_t n_vars = N * state_length + (N - 1) * actuators;
  //size_t n_vars = N*6 + (N - 1)*2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * state_length;
  //size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set the initial variable values
  vars[x_idx]   = x;
  vars[y_idx]   = y;
  vars[psi_idx] = psi;
  vars[v_idx]   = v;
  vars[cte_idx] = cte;
  vars[epsi_idx]= epsi;

  //Limits for state components
  for (int i = 0; i < delta_idx; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  //Steering actuator limits based on simulator steering extremes
  for (int i = delta_idx; i < a_idx; i++) {
    vars_lowerbound[i] = -25*M_PI/180;
    vars_upperbound[i] =  25*M_PI/180;
  }

  //Throttle limits
  for (int i = a_idx; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_idx] = x;
  constraints_lowerbound[y_idx] = y;
  constraints_lowerbound[psi_idx] =psi;
  constraints_lowerbound[v_idx] = v;
  constraints_lowerbound[cte_idx] = cte;
  constraints_lowerbound[epsi_idx] = epsi;

  constraints_upperbound[x_idx] = x;
  constraints_upperbound[y_idx] = y;
  constraints_upperbound[psi_idx] = psi;
  constraints_upperbound[v_idx] = v;
  constraints_upperbound[cte_idx] = cte;
  constraints_upperbound[epsi_idx] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  std::cout << "Solution: " << solution.x.size() << std::endl;
  /**
   * Use future actuation to account for actuator dynamics (100ms delay)
   **/
  result.push_back(solution.x[delta_idx + 1]);
  result.push_back(solution.x[a_idx + 1]);

  for (int i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_idx + i + 1]);
    result.push_back(solution.x[y_idx + i + 1]);
  }

  return result;
}
