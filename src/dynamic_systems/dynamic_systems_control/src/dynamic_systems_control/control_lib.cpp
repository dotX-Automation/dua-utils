/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_control/control_lib.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    void butterworth(ButterworthType type, 
      unsigned int degree, std::vector<double> omegas,
      Polynomiald & num, Polynomiald & den) 
    {
      if(type == ButterworthType::LOW_PASS || type == ButterworthType::HIGH_PASS) {
        if(omegas.size() != 1) {
          std::invalid_argument("Low/High pass filters need only one omega.");
        }
        double omega = omegas.at(0);
        unsigned int l = degree/2;
        Polynomiald tmp;
        tmp.set(2, 1.0);
        den.regrade(0);
        den.set(0, 1.0);
        for(unsigned k = 0; k < l; k++) {
          double exps = double(2*k + degree - 1) / double(2 * degree) * M_PI;
          double re = std::cos(exps);
          double im = std::sin(exps);
          tmp.set(0, (omega*omega) * (re*re + im*im));
          tmp.set(1, -2.0 * omega * re);
          den *= tmp;
        }
        if(degree%2 == 1) {
          tmp.set(0, omega);
          tmp.set(1, 1.0);
          tmp.set(2, 0.0);
          den *= tmp;
        }
        if(type == ButterworthType::HIGH_PASS) {
          num.regrade(degree);
          num.set(degree, 1.0);
        } else {
          num.regrade(0);
          num = std::pow(omega, degree);
        }
      } 
      else if (type == ButterworthType::BAND_PASS) {
        if(omegas.size() != 2) {
          std::invalid_argument("Band pass filters need two omegas.");
        }
        Polynomiald num1, num2; 
        Polynomiald den1, den2;
        butterworth(ButterworthType::HIGH_PASS, degree, {omegas.at(0)}, num1, den1);
        butterworth(ButterworthType::LOW_PASS , degree, {omegas.at(1)}, num2, den2);
        num = num1 * num2;
        den = den1 * den2;
      } 
      else if (type == ButterworthType::NOTCH) {
        double omega = omegas.at(0);
        butterworth(ButterworthType::LOW_PASS , degree, {omegas.at(1)}, num, den);
        num.set(0, 1.0);
        Polynomiald tmp;
        tmp.set(0, 1.0);
        tmp.set(2, omega*omega);
        for(unsigned k = 0; k < degree; k++) {
          num *= tmp;
        }
      } 
      else {
        std::invalid_argument("Unexpected type.");
      }
    }

    
    void realization(
      const Polynomiald & num, const Polynomiald & den,
      MatrixXd & A, MatrixXd & B, MatrixXd & C, MatrixXd & D) 
    {
      unsigned int na = num.size();
      unsigned int nb = den.size();
      
      if(nb > na) {
        std::invalid_argument("The system is not realizable.");
      }

      unsigned int n = na - 1;
      A = MatrixXd::Zero(n, n);
      B = MatrixXd::Zero(n, 1);
      C = MatrixXd::Zero(1, n);
      D = MatrixXd::Zero(1, 1);

      if(n == 0) {
        D(0,0) = num.get(0) / den.get(0);
      } else {
        for(unsigned int i = 0; i < n-1; i++) {
          A(i, i+1) = 1;
        }

        for(unsigned int i = 0; i < n; i++) {
          A(n-1, i) = -den.get(den.degree()-i) / den.get(0);
        }
        B(n-1, 0) = 1;
        
        if(nb < na) {
          for(unsigned int i = 0; i < nb; i++) {
            C(0, i) = num.get(num.degree()-i) / den.get(0);
          }
          D(0,0) = 0;
        } else {
          for(unsigned int i = 0; i < n; i++) {
            C(0, i) = (num.get(num.degree()-i) / num.get(0) - den.get(den.degree()-i) / den.get(0)) * num.get(0) / den.get(0);
          }
          D(0, 0) = num.get(0) / den.get(0);
        }
      }
    }


    void discretization_zoh(
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