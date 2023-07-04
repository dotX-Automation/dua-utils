/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__CONTROL_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__CONTROL_HPP_

#include "visibility_control.h"

#include <polynomial_kit/polynomial.hpp>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace PolynomialKit;

namespace DynamicSystems
{
  namespace Control 
  {
    enum CommonLTIType : unsigned int {
      ORDER_0,
      ORDER_1,
      ORDER_2
    };

    enum ButterworthType : unsigned int {
      LOW_PASS,
      HIGH_PASS,
      BAND_PASS,
      NOTCH
    };

    void DYNAMIC_SYSTEMS_CONTROL_PUBLIC common_lti(CommonLTIType type,
      std::vector<double> params,
      Polynomiald & num, Polynomiald & den);

    void DYNAMIC_SYSTEMS_CONTROL_PUBLIC butterworth(ButterworthType type, 
      unsigned int degree, std::vector<double> omegas,
      Polynomiald & num, Polynomiald & den);

    void DYNAMIC_SYSTEMS_CONTROL_PUBLIC realization(
      const Polynomiald & num, const Polynomiald & den,
      MatrixXd & A, MatrixXd & B, MatrixXd & C, MatrixXd & D);

    void DYNAMIC_SYSTEMS_CONTROL_PUBLIC discretization_zoh(
      double ts, unsigned int steps, 
      const MatrixXd & A, const MatrixXd & B, const MatrixXd & C, const MatrixXd & D,
      MatrixXd & A_zoh, MatrixXd & B_zoh, MatrixXd & C_zoh, MatrixXd & D_zoh);
  }
}

#endif  // DYNAMIC_SYSTEMS_CONTROL__CONTROL_HPP_