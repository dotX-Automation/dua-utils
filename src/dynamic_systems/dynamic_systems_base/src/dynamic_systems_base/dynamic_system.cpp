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
    setup_default();
  }

  System::~System(){
    if(inited_) {
      fini();
    }
  }

  void System::init(std::shared_ptr<InitParams> initParams) {
    if(initParams == nullptr) {
      throw std::invalid_argument("Null init parameters.");
    }
    init_parse(*initParams.get());
    inited_ = true;
    dirty_ = true;
  }

  void System::setup(std::shared_ptr<SetupParams> setupParams) {
    if(setupParams == nullptr) {
      setup_default();
    } else {
      setup_parse(*setupParams.get());
    }
    dirty_ = true;
  }

  void System::fini() {
    if(inited_) {
      deinit();
      inited_ = false;
      dirty_ = true;
    }
  }

  
  bool System::initialized() {
    return inited_;
  }

  bool System::dirty() {
    return dirty_;
  }

  void System::reset(std::shared_ptr<State> state) {
    if(state) {
      reset_ = state->clone();
    }
    state_ = reset_->clone();
    state_validator(*state_.get());
    dirty_ = true;
  }

  void System::input(double in) {
    input(MatrixXd(in));
  }

  void System::input(MatrixXd in){
    input_ = in;
    input_validator(*state_.get(), input_);
    dirty_ = true;
  }

  MatrixXd System::output(){
    update();
    return output_;
  }

  void System::step() {
    std::unique_ptr<State> next = state_->clone();
    dynamic_map(*state_.get(), input_, *next.get());
    state_ = std::move(next);
    state_validator(*state_.get());
    dirty_ = true;
  }

  void System::update() {
    if(dirty_) {
      output_map(*state_.get(), input_, output_);
      dirty_ = false;
    }
  }

  MatrixXd System::evolve(double in) {
    return evolve(MatrixXd(in));
  }

  MatrixXd System::evolve(MatrixXd in){
    input(in);
    MatrixXd res = output();
    step();
    return res;
  }

  std::shared_ptr<State> System::state() {
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

  void System::init_parse(const InitParams& initParams) {
    UNUSED(initParams);
  }

  void System::setup_parse(const SetupParams& setupParams) {
    UNUSED(setupParams);
  }

  void System::setup_default() {}

  void System::deinit() {}

  void System::state_validator(State &state) {
    UNUSED(state);
  }
  
  void System::input_validator(const State &state, MatrixXd &input) {
    UNUSED(state);
    UNUSED(input);
  }

  void System::dynamic_map(const State &state, const MatrixXd &input, State &next) {
    UNUSED(input);
    next.copy(state);
  }

  void System::output_map(const State &state, const MatrixXd &input, MatrixXd& output) {
    UNUSED(state);
    output = input;
  }
}
