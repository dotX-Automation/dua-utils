/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__SAMPLED_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__SAMPLED_HPP_

#include <dynamic_systems_base/dynamic_system.hpp>
#include <dynamic_systems_control/control_lib.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct SampledInitParams : public InitParams {
      virtual ~SampledInitParams();
      virtual std::unique_ptr<InitParams> clone() const;
      virtual void copy(const InitParams &other);

      double time_sampling;
      unsigned long discretization_steps;
    };

    struct SampledSetupParams : public SetupParams {
      virtual ~SampledSetupParams();
      virtual std::unique_ptr<SetupParams> clone() const;
      virtual void copy(const SetupParams &other);
    };

    struct SampledState : public State {
      virtual ~SampledState();
      virtual std::unique_ptr<State> clone() const;
      virtual void copy(const State &other);
    };
    
    class SampledSystem : public System {
      virtual void init(std::shared_ptr<InitParams> initParams);

      public:
        double time_sampling();

      private:
        double time_sampling_ = 0.0;
    };
  }
}

#endif  // DYNAMIC_SYSTEMS_CONTROL__SAMPLED_HPP_