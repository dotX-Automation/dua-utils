/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_control/sampled.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    /* InitParams */
    
    SampledInitParams::~SampledInitParams() {}

    std::unique_ptr<InitParams> SampledInitParams::clone() const {
      std::unique_ptr<SampledInitParams> res = std::make_unique<SampledInitParams>();
      res->copy(*this);
      return res;
    }

    void SampledInitParams::copy(const InitParams &other) {
      InitParams::copy(other);

      auto casted = dynamic_cast<const SampledInitParams&>(other);
      this->time_sampling = casted.time_sampling;
      this->discretization_steps = casted.discretization_steps;
    }


    /* SetupParams */

    SampledSetupParams::~SampledSetupParams() {}

    std::unique_ptr<SetupParams> SampledSetupParams::clone() const {
      std::unique_ptr<SampledSetupParams> res = std::make_unique<SampledSetupParams>();
      res->copy(*this);
      return res;
    }

    void SampledSetupParams::copy(const SetupParams &other) {
      SetupParams::copy(other);
    }


    /* State */

    SampledState::~SampledState() {}

    std::unique_ptr<State> SampledState::clone() const {
      std::unique_ptr<SampledState> res = std::make_unique<SampledState>();
      res->copy(*this);
      return res;
    }

    void SampledState::copy(const State &other) {
      State::copy(other);
    }
    

    /* System */

    void SampledSystem::init(std::shared_ptr<InitParams> initParams) {
      System::init(initParams);

      auto sampledInitParams = std::dynamic_pointer_cast<SampledInitParams>(initParams);
      if (!sampledInitParams) {
        throw std::invalid_argument("Invalid parameters for sampled system.");
      }

      if (sampledInitParams->time_sampling < 0.0) {
        throw std::invalid_argument("Invalid sampling time for sampled system.");
      }
    }

    double SampledSystem::time_sampling() {
      return time_sampling_;
    }
  }
} 