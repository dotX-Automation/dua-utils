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
    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorInitParams : public InitParams<double> {
      ~IntegratorInitParams() override;
      std::unique_ptr<InitParams<double>> clone() const override;
      void copy(const InitParams<double> &other) override;

      double time_sampling;
      unsigned int rows;
      unsigned int cols;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSetupParams : public SetupParams<double> {
      ~IntegratorSetupParams() override;
      std::unique_ptr<SetupParams<double>> clone() const override;
      void copy(const SetupParams<double> &other) override;

      double multiplier;
      double saturation;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorState : public State<double> {
      ~IntegratorState() override;
      std::unique_ptr<State<double>> clone() const override;
      void copy(const State<double> &other) override;

      MatrixX<double> value;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC IntegratorSystem : public System<double> {
      protected:
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL init_parse(const InitParams<double>& initParams) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL setup_parse(const SetupParams<double>& setupParams) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL setup_default() override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL deinit() override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL state_validator(State<double> &state) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL input_validator(const State<double> &state, MatrixX<double> &input) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) override;
        void DYNAMIC_SYSTEMS_CONTROL_LOCAL output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) override;

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