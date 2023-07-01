/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_control/control_lib.hpp>
#include <unsupported/Eigen/Polynomials>

namespace DynamicSystems
{
  namespace Control 
  {
    void distretization_zoh(
      double time_sampling, unsigned int zoh_steps, 
      const MatrixXd & A, const MatrixXd & B, const MatrixXd & C, const MatrixXd & D,
      MatrixXd & A_zoh, MatrixXd & B_zoh, MatrixXd & C_zoh, MatrixXd & D_zoh)
    {
      if(time_sampling <= 0.0) {
        throw std::invalid_argument("Invalid time sampling.");
      }

      if(zoh_steps == 0) {
        throw std::invalid_argument("Invalid zoh steps.");
      }

      double dt = time_sampling / double(zoh_steps);
      MatrixXd H = MatrixXd::Zero(A.rows(), A.cols());
      MatrixXd H_prev = MatrixXd::Identity(A.rows(), A.cols());
      MatrixXd H_next = MatrixXd::Zero(A.rows(), A.cols());
      for(unsigned int i = 1; i <= zoh_steps; i++) {
        double t = i*dt;
        H_next = (A*t).exp();
        H += (0.5 * (H_prev + H_next)) * dt;
        H_prev = H_next;
      }

      A_zoh = (A*time_sampling).exp();
      B_zoh = H*B;
      C_zoh = C;
      D_zoh = D;
    }
  }
} 