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

#include "visibility_control.h"

#include <dynamic_systems_base/dynamic_system.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorInitParams : public InitParams {
      ~IntegratorInitParams() override;
      std::unique_ptr<InitParams> clone() const override;
      void copy(const InitParams &other) override;

      double time_sampling;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSetupParams : public SetupParams {
      ~IntegratorSetupParams() override;
      std::unique_ptr<SetupParams> clone() const override;
      void copy(const SetupParams &other) override;

      double coeff;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorState : public State {
      ~IntegratorState() override;
      std::unique_ptr<State> clone() const override;
      void copy(const State &other) override;

      MatrixXd x;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSystem : public System {
      public:
        IntegratorSystem();
        ~IntegratorSystem() override;

        void init(std::shared_ptr<InitParams> initParams) override;
        void setup(std::shared_ptr<SetupParams> setupParams) override;
        void fini() override;

      protected:
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL state_validator(std::unique_ptr<State> &state) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL input_validator(MatrixXd &input) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL dynamic_map(std::unique_ptr<State> &state, MatrixXd &input, std::unique_ptr<State> &next) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL output_map(std::unique_ptr<State> &state, MatrixXd &input, MatrixXd& output) override;

      private:
        double kt_ = 1.0;
        double coeff_ = 1.0;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_