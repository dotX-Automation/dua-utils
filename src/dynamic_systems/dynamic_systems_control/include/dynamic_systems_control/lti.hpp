/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__LTI_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__LTI_HPP_

#include "visibility_control.h"

#include <dynamic_systems_base/dynamic_system.hpp>
#include <dynamic_systems_control/control_lib.hpp>

namespace DynamicSystems
{
  namespace Control 
  {
    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTIInitParams : public InitParams<double> {
      ~LTIInitParams() override;
      std::unique_ptr<InitParams<double>> clone() const override;
      void copy(const InitParams<double> &other) override;

      double time_sampling;
      unsigned int zoh_steps;
      MatrixX<double> matrixA;
      MatrixX<double> matrixB;
      MatrixX<double> matrixC;
      MatrixX<double> matrixD;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTISetupParams : public SetupParams<double> {
      ~LTISetupParams() override;
      std::unique_ptr<SetupParams<double>> clone() const override;
      void copy(const SetupParams<double> &other) override;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTIState : public State<double> {
      ~LTIState() override;
      std::unique_ptr<State<double>> clone() const override;
      void copy(const State<double> &other) override;

      MatrixXd value;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTISystem : public System<double> {
      public:
        static void make_common_lti(
          LTISystem & lti,
          double time_sampling, unsigned int zoh_steps, 
          CommonLTIType type, std::vector<double> params);
        void make_common_lti(
          double time_sampling, unsigned int zoh_steps, 
          CommonLTIType type, std::vector<double> params);

        static void make_butterworth(
          LTISystem & lti,
          double time_sampling, unsigned int zoh_steps, 
          ButterworthType type, unsigned int degree, std::vector<double> omegas);
        void make_butterworth(
          double time_sampling, unsigned int zoh_steps, 
          ButterworthType type, unsigned int degree, std::vector<double> omegas);

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
        unsigned int n_;
        unsigned int m_;
        unsigned int q_;
        MatrixXd A_;
        MatrixXd B_;
        MatrixXd C_;
        MatrixXd D_;

        /* setup members */
    };
  }
}

#endif // DYNAMIC_SYSTEMS_CONTROL__LTI_HPP_