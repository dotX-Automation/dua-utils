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

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

namespace DynamicSystems
{
  namespace Control 
  {
    void DYNAMIC_SYSTEMS_CONTROL_PUBLIC distretization_zoh(
      double ts, unsigned int steps, 
      const MatrixXd & A, const MatrixXd & B, const MatrixXd & C, const MatrixXd & D,
      MatrixXd & A_zoh, MatrixXd & B_zoh, MatrixXd & C_zoh, MatrixXd & D_zoh);
  }
}

#endif  // DYNAMIC_SYSTEMS_CONTROL__CONTROL_HPP_