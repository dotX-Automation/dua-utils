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

#include <dynamic_systems_base/dynamic_system.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct IntegratorInitParams : public InitParams {
      ~IntegratorInitParams() override;
      std::unique_ptr<InitParams> clone() const override;
      void copy(const InitParams &other) override;

      double time_sampling;
    };

    struct IntegratorSetupParams : public SetupParams {
      ~IntegratorSetupParams() override;
      std::unique_ptr<SetupParams> clone() const override;
      void copy(const SetupParams &other) override;
    };

    struct IntegratorState : public State {
      ~IntegratorState() override;
      std::unique_ptr<State> clone() const override;
      void copy(const State &other) override;

      double x;
    };
    
    class IntegratorSystem : public System {
      public:
        IntegratorSystem();
        ~IntegratorSystem() override;

        void init(std::shared_ptr<InitParams> initParams) override;
        void setup(std::shared_ptr<SetupParams> setupParams) override;
        void fini() override;

      protected:
        void state_validator(std::unique_ptr<State> &state) override;
        void input_validator(MatrixXd &input) override;
        void dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next) override;
        void output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output) override;

      private:
        double kt;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_