/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 29, 2023
 */

#ifndef DYNAMIC_SYSTEMS_FILTERS__INTEGRATOR_HPP_
#define DYNAMIC_SYSTEMS_FILTERS__INTEGRATOR_HPP_

#include <dynamic_systems/visibility_control.h>

#include <dynamic_systems/base/system.hpp>

using namespace DynamicSystems::Base;

namespace DynamicSystems
{
  namespace Filters 
  {
    struct DYNAMIC_SYSTEMS_PUBLIC JumpFilterInitParams : public InitParams<double> {
      ~JumpFilterInitParams() override;
      std::unique_ptr<InitParams<double>> clone() const override;
      void copy(const InitParams<double> &other) override;

      bool element_wise;
      unsigned int rows;
      unsigned int cols;
    };

    struct DYNAMIC_SYSTEMS_PUBLIC JumpFilterSetupParams : public SetupParams<double> {
      ~JumpFilterSetupParams() override;
      std::unique_ptr<SetupParams<double>> clone() const override;
      void copy(const SetupParams<double> &other) override;

      double evol_diff;
      double jump_diff;
    };

    struct DYNAMIC_SYSTEMS_PUBLIC JumpFilterState : public State<double> {
      ~JumpFilterState() override;
      std::unique_ptr<State<double>> clone() const override;
      void copy(const State<double> &other) override;

      MatrixX<double> value;
      MatrixX<bool> jumping;
    };
    
    class DYNAMIC_SYSTEMS_PUBLIC JumpFilterSystem : public System<double> {
      protected:
        void DYNAMIC_SYSTEMS_LOCAL init_parse(const InitParams<double>& initParams) override;
        void DYNAMIC_SYSTEMS_LOCAL setup_parse(const SetupParams<double>& setupParams) override;
        void DYNAMIC_SYSTEMS_LOCAL setup_default() override;
        void DYNAMIC_SYSTEMS_LOCAL deinit() override;
        void DYNAMIC_SYSTEMS_LOCAL state_validator(State<double> &state) override;
        void DYNAMIC_SYSTEMS_LOCAL input_validator(const State<double> &state, MatrixX<double> &input) override;
        void DYNAMIC_SYSTEMS_LOCAL dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) override;
        void DYNAMIC_SYSTEMS_LOCAL output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) override;

      private:
        /* init members */
        bool elem_wise_;

        /* setup members */
        double evol_diff_;
        double jump_diff_;
    };
  }
}

#endif // DYNAMIC_SYSTEMS_FILTERS__INTEGRATOR_HPP_