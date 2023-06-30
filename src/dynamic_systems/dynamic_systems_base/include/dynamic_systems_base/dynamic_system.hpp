/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_BASE__DYNAMIC_SYSTEM_HPP_
#define DYNAMIC_SYSTEMS_BASE__DYNAMIC_SYSTEM_HPP_

#include "visibility_control.h"

#include <cmath>
#include <memory>
#include <Eigen/Geometry>

#define UNUSED(arg) (void)(arg)

using namespace Eigen;

namespace DynamicSystems
{
  /**
   * Holds init parameters for a dynamic system. Made to be derived from.
   */
  struct DYNAMIC_SYSTEMS_BASE_PUBLIC InitParams
  {
    virtual ~InitParams();
    virtual std::unique_ptr<InitParams> clone() const;
    virtual void copy(const InitParams &other);
  };

  /**
   * Holds setup parameters for a dynamic system. Made to be derived from.
   */
  struct DYNAMIC_SYSTEMS_BASE_PUBLIC SetupParams
  {
    virtual ~SetupParams();
    virtual std::unique_ptr<SetupParams> clone() const;
    virtual void copy(const SetupParams &other);
  };

  /**
   * Holds internal state for a dynamic system. Made to be derived from.
   */
  struct DYNAMIC_SYSTEMS_BASE_PUBLIC State
  {
    virtual ~State();
    virtual std::unique_ptr<State> clone() const;
    virtual void copy(const State &other);
  };

  /**
   * Base class for dynamic systems.
   */
  class DYNAMIC_SYSTEMS_BASE_PUBLIC System
  {
  public:
    System();
    virtual ~System();

    virtual void init(std::shared_ptr<InitParams> initParams);
    virtual void setup(std::shared_ptr<SetupParams> setupParams);
    virtual void fini();
    
    void reset(std::shared_ptr<State> state = nullptr);
    void input(MatrixXd in);
    MatrixXd output();
    void step();
    MatrixXd evolve(MatrixXd in);

    std::unique_ptr<State> state();
    std::array<unsigned int, 2u> input_size();
    std::array<unsigned int, 2u> output_size();

  protected:
    virtual void DYNAMIC_SYSTEMS_BASE_LOCAL state_validator(std::unique_ptr<State> &state);
    virtual void DYNAMIC_SYSTEMS_BASE_LOCAL input_validator(MatrixXd &input);
    virtual void DYNAMIC_SYSTEMS_BASE_LOCAL dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next);
    virtual void DYNAMIC_SYSTEMS_BASE_LOCAL output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output);

  private:
    bool dirty_;
    std::unique_ptr<State> reset_;
    std::unique_ptr<State> state_;
    MatrixXd input_;
    MatrixXd output_;
  };
}

#endif // DYNAMIC_SYSTEMS_BASE__DYNAMIC_SYSTEM_HPP_
