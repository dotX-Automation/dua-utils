/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_control/lti.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    /* InitParams */

    LTIInitParams::~LTIInitParams() {}

    std::unique_ptr<InitParams> LTIInitParams::clone() const {
      std::unique_ptr<LTIInitParams> res = std::make_unique<LTIInitParams>();
      res->copy(*this);
      return res;
    }

    void LTIInitParams::copy(const InitParams &other) {
      InitParams::copy(other);
      auto casted = dynamic_cast<const LTIInitParams&>(other);
      this->time_sampling = casted.time_sampling;
      this->zoh_steps = casted.zoh_steps;
      this->matrixA = casted.matrixA;
      this->matrixB = casted.matrixB;
      this->matrixC = casted.matrixC;
      this->matrixD = casted.matrixD;
    }


    /* SetupParams */

    LTISetupParams::~LTISetupParams() {}

    std::unique_ptr<SetupParams> LTISetupParams::clone() const {
      std::unique_ptr<LTISetupParams> res = std::make_unique<LTISetupParams>();
      res->copy(*this);
      return res;
    }

    void LTISetupParams::copy(const SetupParams &other) {
      SetupParams::copy(other);
    }


    /* State */

    LTIState::~LTIState() {}

    std::unique_ptr<State> LTIState::clone() const {
      std::unique_ptr<LTIState> res = std::make_unique<LTIState>();
      res->copy(*this);
      return res;
    }

    void LTIState::copy(const State &other) {
      State::copy(other);
      auto casted = dynamic_cast<const LTIState&>(other);
      this->value = casted.value;
    }
    

    /* System */
    LTISystem& LTISystem::make_common_lti(double time_sampling, unsigned int zoh_steps, 
      CommonLTIType type, std::vector<double> params) 
    {
      Polynomiald num, den;
      std::shared_ptr<LTIInitParams> initParams = std::make_shared<LTIInitParams>();
      LTISystem lti;

      initParams->time_sampling = time_sampling;
      initParams->zoh_steps = zoh_steps;
      common_lti(type, params, num, den);
      realization(num, den, initParams->matrixA, initParams->matrixB, initParams->matrixC, initParams->matrixD);

      lti.init(initParams);
      return lti;
    }

    LTISystem& LTISystem::make_butterworth(double time_sampling, unsigned int zoh_steps, 
      ButterworthType type, unsigned int degree, std::vector<double> omegas)
    {
      Polynomiald num, den;
      std::shared_ptr<LTIInitParams> initParams = std::make_shared<LTIInitParams>();
      LTISystem lti;

      initParams->time_sampling = time_sampling;
      initParams->zoh_steps = zoh_steps;
      butterworth(type, degree, omegas, num, den);
      realization(num, den, initParams->matrixA, initParams->matrixB, initParams->matrixC, initParams->matrixD);

      lti.init(initParams);
      return lti;
    }

    void LTISystem::init_parse(const InitParams& initParams) {
      auto casted = dynamic_cast<const LTIInitParams&>(initParams);

      if(casted.time_sampling < 0.0) {
        throw std::invalid_argument("Invalid time sampling for lti system.");
      }

      if(casted.time_sampling > 0.0 && casted.zoh_steps == 0) {
        throw std::invalid_argument("Invalid zoh steps for lti system.");
      }

      if(casted.matrixA.rows() != casted.matrixA.cols() ||
         casted.matrixB.rows() != casted.matrixA.rows() ||
         casted.matrixC.cols() != casted.matrixA.cols() ||
         casted.matrixD.cols() != casted.matrixB.cols() ||
         casted.matrixD.rows() != casted.matrixC.rows()) 
      {
        throw std::invalid_argument("Invalid matrices size for lti system.");
      }

      n_ = casted.matrixA.rows();
      m_ = casted.matrixB.cols();
      q_ = casted.matrixC.rows();

      if(casted.time_sampling > 0.0) {
        discretization_zoh(casted.time_sampling, casted.zoh_steps,
          casted.matrixA, casted.matrixB, casted.matrixC, casted.matrixD,
          A_, B_, C_, D_);
      } else {
        A_ = casted.matrixA;
        B_ = casted.matrixB;
        C_ = casted.matrixC;
        D_ = casted.matrixD;
      }
      
      std::shared_ptr<LTIState> state = std::make_shared<LTIState>();
      state->value = MatrixXd(n_, 1);
      reset(state);
      input(MatrixXd(m_, 1));
      update();
    }

    void LTISystem::setup_parse(const SetupParams& setupParams) {
      UNUSED(setupParams);
    }

    void LTISystem::setup_default() {}

    void LTISystem::deinit(){}

    void LTISystem::state_validator(State &state) {
      const LTIState &state_casted = static_cast<const LTIState&>(state);
      if(state_casted.value.rows() != n_ || state_casted.value.cols() != 1) {
        throw std::invalid_argument("Invalid state size.");
      }
    }

    void LTISystem::input_validator(const State &state, MatrixXd &input) {
      UNUSED(state);
      if(input.rows() != m_ || input.cols() != 1) {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void LTISystem::dynamic_map(const State &state, const MatrixXd &input, State &next) {
      const LTIState &state_casted = static_cast<const LTIState&>(state);
      LTIState &next_casted = static_cast<LTIState&>(next);
      next_casted.value = A_ * state_casted.value + B_ * input;
    }

    void LTISystem::output_map(const State &state, const MatrixXd &input, MatrixXd& output) {
      const LTIState &state_casted = static_cast<const LTIState&>(state);
      output = C_ * state_casted.value + D_ * input;
    }
  }
} 