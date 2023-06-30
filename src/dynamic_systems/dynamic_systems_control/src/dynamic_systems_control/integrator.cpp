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
      InitParams::copy(other);
      auto casted = dynamic_cast<const IntegratorInitParams&>(other);
      this->time_sampling = casted.time_sampling;
    }


    /* SetupParams */

    IntegratorSetupParams::~IntegratorSetupParams() {}

    std::unique_ptr<SetupParams> IntegratorSetupParams::clone() const {
      std::unique_ptr<IntegratorSetupParams> res = std::make_unique<IntegratorSetupParams>();
      res->copy(*this);
      return res;
    }

    void IntegratorSetupParams::copy(const SetupParams &other) {
      SetupParams::copy(other);
      auto casted = dynamic_cast<const IntegratorSetupParams&>(other);
      this->coeff = casted.coeff;
    }


    /* State */

    IntegratorState::~IntegratorState() {}

    std::unique_ptr<State> IntegratorState::clone() const {
      std::unique_ptr<IntegratorState> res = std::make_unique<IntegratorState>();
      res->copy(*this);
      return res;
    }

    void IntegratorState::copy(const State &other) {
      State::copy(other);

      auto casted = dynamic_cast<const IntegratorState&>(other);
      this->x = casted.x;
    }
    

    /* System */

    IntegratorSystem::IntegratorSystem() {
      std::shared_ptr<IntegratorState> state = std::make_shared<IntegratorState>();
      state->x = MatrixXd(1,1);
      reset(state);
    }

    IntegratorSystem::~IntegratorSystem() {}

    void IntegratorSystem::init(std::shared_ptr<InitParams> initParams) {
      System::init(initParams);
      
      auto casted = std::dynamic_pointer_cast<IntegratorInitParams>(initParams);
      if(!casted) {
        throw std::invalid_argument("Invalid init parameters for integrator system.");
      }
      if(casted->time_sampling < 0.0) {
        throw std::invalid_argument("Invalid init parameters for integrator system.");
      }

      if(casted->time_sampling > 0.0) {
        this->kt_ = casted->time_sampling;
      } else {
        this->kt_ = 1.0;
      }
    }

    void IntegratorSystem::setup(std::shared_ptr<SetupParams> setupParams) {
      System::setup(setupParams);

      auto casted = std::dynamic_pointer_cast<IntegratorSetupParams>(setupParams);
      if(!casted) {
        throw std::invalid_argument("Invalid setup parameters for integrator system.");
      }
      
      this->coeff_ = casted->coeff;
    }

    void IntegratorSystem::fini(){
      System::fini();
    }

    void IntegratorSystem::state_validator(std::unique_ptr<State> &state) {
      UNUSED(state);
    }

    void IntegratorSystem::input_validator(MatrixXd &input) {
      UNUSED(input);
    }

    void IntegratorSystem::dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next) {
      IntegratorState *state_ptr = (IntegratorState *) state.get();
      IntegratorState *next_ptr = (IntegratorState *) next.get();
      next_ptr->x = state_ptr->x + this->coeff_ * this->kt_ * input;
    }

    void IntegratorSystem::output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output) {
      UNUSED(input);
      IntegratorState *state_ptr = (IntegratorState *) state.get();
      output = state_ptr->x;
    }
  }
} 