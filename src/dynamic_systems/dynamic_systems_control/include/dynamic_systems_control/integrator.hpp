/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_

#include <dynamic_systems_control/sampled.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct IntegratorInitParams : public SampledInitParams {
      virtual ~IntegratorInitParams();
      virtual std::unique_ptr<InitParams> clone() const;
      virtual void copy(const InitParams &other);
    };

    struct IntegratorSetupParams : public SampledSetupParams {
      virtual ~IntegratorSetupParams();
      virtual std::unique_ptr<SetupParams> clone() const;
      virtual void copy(const SetupParams &other);
    };

    struct IntegratorState : public SampledState {
      virtual ~IntegratorState();
      virtual std::unique_ptr<State> clone() const;
      virtual void copy(const State &other);

      double x;
    };
    
    class IntegratorSystem : public SampledSystem {
      virtual void init(std::shared_ptr<InitParams> initParams);

      virtual void dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next);
      virtual void output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output);

      private:
        double kt;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_