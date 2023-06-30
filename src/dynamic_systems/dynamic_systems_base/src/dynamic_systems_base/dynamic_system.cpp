/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#include <dynamic_systems_base/dynamic_system.hpp>


namespace DynamicSystems
{
  /* InitParams */

  InitParams::~InitParams() {}

  std::unique_ptr<InitParams> InitParams::clone() const {
    return std::make_unique<InitParams>();
  }

  void InitParams::copy(const InitParams &other) {
    UNUSED(other);
  }


  /* SetupParams */

  SetupParams::~SetupParams() {}

  std::unique_ptr<SetupParams> SetupParams::clone() const {
    return std::make_unique<SetupParams>();
  }

  void SetupParams::copy(const SetupParams &other) {
    UNUSED(other);
  }


  /* State */

  State::~State() {}

  std::unique_ptr<State> State::clone() const {
    return std::make_unique<State>();
  }

  void State::copy(const State &other) {
    UNUSED(other);
  }


  /* System */

  System::System() {
    reset_ = std::make_unique<State>();
    state_ = std::make_unique<State>();
  }

  System::~System(){  }

  void System::init(std::shared_ptr<InitParams> initParams) {
    if(initParams == nullptr) {
      throw std::invalid_argument("Null init parameters.");
    }
  }

  void System::setup(std::shared_ptr<SetupParams> setupParams) {
    if(setupParams == nullptr) {
      throw std::invalid_argument("Null setup parameters.");
    }
  }

  void System::fini() {}

  
  void System::reset(std::shared_ptr<State> state) {
    if(state) {
      reset_ = state->clone();
    }
    state_ = reset_->clone();
    state_validator(state_);
    dirty_ = true;
  }

  void System::input(MatrixXd in){
    input_ = in;
    input_validator(input_);
    dirty_ = true;
  }

  MatrixXd System::output(){
    if(dirty_) {
      output_map(state_, input_, output_);
      dirty_ = false;
    }
    return output_;
  }

  void System::step() {
    std::unique_ptr<State> next = state_->clone();
    dynamic_map(state_, input_, next);
    state_ = std::move(next);
    state_validator(state_);
    dirty_ = true;
  }

  MatrixXd System::evolve(MatrixXd in){
    input(in);
    MatrixXd res = output();
    step();
    return res;
  }

  
  std::unique_ptr<State> System::state() {
    return state_->clone();
  }

  std::array<unsigned int, 2u> System::input_size(){
    std::array<unsigned int, 2u> size;
    size[0] = input_.rows();
    size[1] = input_.cols();
    return size;
  }

  std::array<unsigned int, 2u> System::output_size(){
    std::array<unsigned int, 2u> size;
    MatrixXd output_ = output();
    size[0] = output_.rows();
    size[1] = output_.cols();
    return size;
  }

  void System::state_validator(std::unique_ptr<State> &state) {
    UNUSED(state);
  }
  
  void System::input_validator(MatrixXd &input) {
    UNUSED(input);
  }

  void System::dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next) {
    UNUSED(input);
    next = state->clone();
  }

  void output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output) {
    UNUSED(state);
    output = input;
  }
}
