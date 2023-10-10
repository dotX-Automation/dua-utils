/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems/filters/jump_filter.hpp>

namespace DynamicSystems
{
  namespace Filters 
  {
    /* InitParams */

    JumpFilterInitParams::~JumpFilterInitParams() {}

    std::unique_ptr<InitParams<double>> JumpFilterInitParams::clone() const {
      std::unique_ptr<JumpFilterInitParams> res = std::make_unique<JumpFilterInitParams>();
      res->copy(*this);
      return res;
    }

    void JumpFilterInitParams::copy(const InitParams<double> &other) {
      InitParams<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterInitParams&>(other);
      this->element_wise = casted.element_wise;
      this->rows = casted.rows;
      this->cols = casted.cols;
    }


    /* SetupParams */

    JumpFilterSetupParams::~JumpFilterSetupParams() {}

    std::unique_ptr<SetupParams<double>> JumpFilterSetupParams::clone() const {
      std::unique_ptr<JumpFilterSetupParams> res = std::make_unique<JumpFilterSetupParams>();
      res->copy(*this);
      return res;
    }

    void JumpFilterSetupParams::copy(const SetupParams<double> &other) {
      SetupParams<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterSetupParams&>(other);
      this->slope = casted.slope;
    }


    /* State */

    JumpFilterState::~JumpFilterState() {}

    std::unique_ptr<State<double>> JumpFilterState::clone() const {
      std::unique_ptr<JumpFilterState> res = std::make_unique<JumpFilterState>();
      res->copy(*this);
      return res;
    }

    void JumpFilterState::copy(const State<double> &other) {
      State<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterState&>(other);
      this->value = casted.value;
    }
    

    /* System */

    void JumpFilterSystem::init_parse(const InitParams<double>& initParams) {
      auto casted = dynamic_cast<const JumpFilterInitParams&>(initParams);

      this->elemwise_ = casted.element_wise;

      std::shared_ptr<JumpFilterState> state = std::make_shared<JumpFilterState>();
      state->value = MatrixXd(casted.rows, casted.cols);
      reset(state);
      input(MatrixXd(casted.rows, casted.cols));
      update();
    }

    void JumpFilterSystem::setup_parse(const SetupParams<double>& setupParams) {
      auto casted = dynamic_cast<const JumpFilterSetupParams&>(setupParams);

      if(casted.slope < 0.0) {
        throw std::invalid_argument("Invalid saturation for integrator system.");
      }
      
      this->slope_ = casted.slope;
    }

    void JumpFilterSystem::setup_default() {
      this->slope_ = 0.0;
    }

    void JumpFilterSystem::deinit(){}

    void JumpFilterSystem::state_validator(State<double> &state) {
      //JumpFilterState &state_casted = static_cast<JumpFilterState&>(state);
      UNUSED(state);
    }

    void JumpFilterSystem::input_validator(const State<double> &state, MatrixX<double> &input) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      if(input.rows() != state_casted.value.rows() || input.cols() != state_casted.value.cols()) {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void JumpFilterSystem::dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      JumpFilterState &next_casted = static_cast<JumpFilterState&>(next);
      
      MatrixX<double> diff = input - state_casted.value;
      
      if(this->elemwise_) {
        for(unsigned int r = 0; r < diff.rows(); r++) {
          for(unsigned int c = 0; c < diff.rows(); c++) {
            double sign = ((diff(r, c) < 0.0) ? -1.0 : 1.0);
            diff(r, c) = sign * std::min(std::abs(diff(r, c)), slope_);
          }
        }
      } else {
        if(diff.norm() > this->slope_) {
          diff *= (this->slope_ / diff.norm());
        }
      }

      next_casted.value = state_casted.value + diff;
    }

    void JumpFilterSystem::output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) {
      UNUSED(input);
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      output = state_casted.value;
    }
  }
} 