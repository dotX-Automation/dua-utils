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
    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTIInitParams : public InitParams {
      ~LTIInitParams() override;
      std::unique_ptr<InitParams> clone() const override;
      void copy(const InitParams &other) override;

      double time_sampling;
      unsigned int zoh_steps;
      MatrixXd matrixA;
      MatrixXd matrixB;
      MatrixXd matrixC;
      MatrixXd matrixD;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTISetupParams : public SetupParams {
      ~LTISetupParams() override;
      std::unique_ptr<SetupParams> clone() const override;
      void copy(const SetupParams &other) override;
    };

    struct DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTIState : public State {
      ~LTIState() override;
      std::unique_ptr<State> clone() const override;
      void copy(const State &other) override;

      MatrixXd value;
    };
    
    class DYNAMIC_SYSTEMS_CONTROL_PUBLIC LTISystem : public System {
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