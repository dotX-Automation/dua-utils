/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_control/integrator.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    /* InitParams */
    IntegratorInitParams::~IntegratorInitParams() {}

    std::unique_ptr<InitParams> IntegratorInitParams::clone() const {
      std::unique_ptr<IntegratorInitParams> res = std::make_unique<IntegratorInitParams>();
      res->copy(*this);
      return res;
    }

    void IntegratorInitParams::copy(const InitParams &other) {
      SampledInitParams::copy(other);
    }

    /* SetupParams */
    IntegratorSetupParams::~IntegratorSetupParams() {}

    std::unique_ptr<SetupParams> IntegratorSetupParams::clone() const {
      std::unique_ptr<IntegratorSetupParams> res = std::make_unique<IntegratorSetupParams>();
      res->copy(*this);
      return res;
    }

    void IntegratorSetupParams::copy(const SetupParams &other) {
      SampledSetupParams::copy(other);
    }

    /* State */
    IntegratorState::~IntegratorState() {}

    std::unique_ptr<State> IntegratorState::clone() const {
      std::unique_ptr<IntegratorState> res = std::make_unique<IntegratorState>();
      res->copy(*this);
      return res;
    }

    void IntegratorState::copy(const State &other) {
      SampledState::copy(other);

      auto casted = dynamic_cast<const IntegratorState&>(other);
      this->x = casted.x;
    }
    
    /* System */

    void IntegratorSystem::init(std::shared_ptr<InitParams> initParams) {
      
    }

    void IntegratorSystem::dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next) {
      IntegratorState *state_ptr = (IntegratorState *) state.get();
      IntegratorState *next_ptr = (IntegratorState *) state.get();
      next_ptr->x = state_ptr->x + input(0,0);
    }

    void IntegratorSystem::output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output) {
      IntegratorState *state_ptr = (IntegratorState *) state.get();
      output(0,0) = state_ptr->x;
    }
  }
} 