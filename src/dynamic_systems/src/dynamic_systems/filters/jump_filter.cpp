/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems/filters/jump_filter.hpp>

#include <iostream>

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
      this->evol_diff = casted.evol_diff;
      this->jump_diff = casted.jump_diff;
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
      this->jumping = casted.jumping;
    }
    

    /* System */

    void JumpFilterSystem::init_parse(const InitParams<double>& initParams) {
      auto casted = dynamic_cast<const JumpFilterInitParams&>(initParams);

      this->elem_wise_ = casted.element_wise;

      std::shared_ptr<JumpFilterState> state = std::make_shared<JumpFilterState>();
      state->value = MatrixX<double>::Zero(casted.rows, casted.cols);
      if(this->elem_wise_) {
        state->jumping = MatrixX<bool>::Zero(casted.rows, casted.cols);
      } else {
        state->jumping = MatrixX<bool>::Zero(1, 1);
      }

      reset(state);
      input(MatrixX<double>(casted.rows, casted.cols));
      update();
    }

    void JumpFilterSystem::setup_parse(const SetupParams<double>& setupParams) {
      auto casted = dynamic_cast<const JumpFilterSetupParams&>(setupParams);

      if(casted.evol_diff < 0.0) {
        throw std::invalid_argument("Invalid evolution difference for jump filter system.");
      }

      if(casted.jump_diff < 0.0) {
        throw std::invalid_argument("Invalid jumping difference for jump filter system.");
      }

      this->evol_diff_ = casted.evol_diff;
      this->jump_diff_ = casted.jump_diff;
    }

    void JumpFilterSystem::setup_default() {
      this->evol_diff_ = 0.0;
      this->jump_diff_ = 0.0;
    }

    void JumpFilterSystem::deinit(){}

    void JumpFilterSystem::state_validator(State<double> &state) {
      JumpFilterState &state_casted = static_cast<JumpFilterState&>(state);
      if(this->elem_wise_) {
        if(state_casted.value.rows() != state_casted.jumping.rows() ||
           state_casted.value.cols() != state_casted.jumping.cols()) 
        {
          throw std::invalid_argument("Invalid state size.");
        }
      } else {
        if(state_casted.jumping.rows() != 1 || 
           state_casted.jumping.cols() != 1) 
        {
          throw std::invalid_argument("Invalid state size.");
        }
      }
    }

    void JumpFilterSystem::input_validator(const State<double> &state, MatrixX<double> &input) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      if(input.rows() != state_casted.value.rows() || 
         input.cols() != state_casted.value.cols()) 
      {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void JumpFilterSystem::dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      JumpFilterState &next_casted = static_cast<JumpFilterState&>(next);
      
      MatrixX<double> diff = input - state_casted.value;
      MatrixX<bool> jumping = state_casted.jumping;

      if(this->elem_wise_) {
        for(unsigned int r = 0; r < diff.rows(); r++) {
          for(unsigned int c = 0; c < diff.rows(); c++) {
            double s = ((diff(r, c) < 0.0) ? -1.0 : 1.0);
            double d = std::abs(diff(r, c));
            if(jumping(r,c)) {
              jumping(r,c) = diff(r,c) > this->evol_diff_;
              diff(r,c) = s * std::min(d, evol_diff_);
            } else if(diff(r,c) >= this->jump_diff_) {
              jumping(r,c) = true;
              diff(r,c) = s * std::min(d, evol_diff_);
            }
          }
        }
      } else {
        double nrm = diff.norm();
        if(jumping(0,0)) {
          jumping(0,0) = nrm > this->evol_diff_;
          if(nrm > this->evol_diff_) diff *= (this->evol_diff_ / nrm);
        } else if (nrm >= jump_diff_) {
          jumping(0,0) = true;
          if(nrm > this->evol_diff_) diff *= (this->evol_diff_ / nrm);
        }
      }

      next_casted.value = state_casted.value + diff;
      next_casted.jumping = jumping; 
    }

    void JumpFilterSystem::output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) {
      UNUSED(input);
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      output = state_casted.value;
    }
  }
} 