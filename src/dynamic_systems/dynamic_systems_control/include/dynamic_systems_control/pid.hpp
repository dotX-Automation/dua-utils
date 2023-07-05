/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 5, 2023
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__PID_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__PID_HPP_

#include "visibility_control.h"

#include <dynamic_systems_base/dynamic_system.hpp>
#include <dynamic_systems_control/control_lib.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC PIDInitParams : public InitParams<double> {
      ~PIDInitParams() override;
      std::unique_ptr<InitParams<double>> clone() const override;
      void copy(const InitParams<double> &other) override;

      double time_sampling;
      double error_deadzone;
      double integral_saturation;
      double integral_reset_error_threshold;
      double integral_reset_divider;
      double integral_reset_value;
      double derivative_filter_pole;
      bool bumpless;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC PIDSetupParams : public SetupParams<double> {
      ~PIDSetupParams() override;
      std::unique_ptr<SetupParams<double>> clone() const override;
      void copy(const SetupParams<double> &other) override;

      double k_p;
      double k_i;
      double k_d;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC PIDState : public State<double> {
      ~PIDState() override;
      std::unique_ptr<State<double>> clone() const override;
      void copy(const State<double> &other) override;

      double x_i;
      double x_d;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC PIDSystem : public System<double> {
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
        double ts_;
        double err_deadzone_;
        double int_sat_;
        double int_reset_thr_;
        double int_reset_div_;
        double int_reset_val_;
        double der_pole_;
        bool bumpless_;
        double A_;
        double B_;
        double C_;
        double D_;

        /* setup members */
        double kp_;
        double ki_;
        double kd_;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__PID_HPP_