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
      unsigned int rows;
      unsigned int cols;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSetupParams : public SetupParams {
      ~IntegratorSetupParams() override;
      std::unique_ptr<SetupParams> clone() const override;
      void copy(const SetupParams &other) override;

      double multiplier;
      double saturation;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorState : public State {
      ~IntegratorState() override;
      std::unique_ptr<State> clone() const override;
      void copy(const State &other) override;

      MatrixXd value;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSystem : public System {
      protected:
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL init_parse(const InitParams& initParams) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL setup_parse(const SetupParams& setupParams) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL setup_default() override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL deinit() override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL state_validator(State &state) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL input_validator(const State &state, MatrixXd &input) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL dynamic_map(const State &state, const MatrixXd &input, State &next) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL output_map(const State &state, const MatrixXd &input, MatrixXd& output) override;

      private:
        /* init members */
        double kt_;

        /* setup members */
        double mul_;
        double sat_;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__INTEGRATOR_HPP_