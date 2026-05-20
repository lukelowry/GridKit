#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Components/LoadRL/LoadRLData.hpp>
#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class LoadRL final : public EMT::Component<ScalarT, IdxT>
    {
      using Component<ScalarT, IdxT>::gridkit_component_id_;
      using Component<ScalarT, IdxT>::size_;
      using Component<ScalarT, IdxT>::y_;
      using Component<ScalarT, IdxT>::yp_;
      using Component<ScalarT, IdxT>::tag_;
      using Component<ScalarT, IdxT>::f_;
      using Component<ScalarT, IdxT>::variable_indices_;
      using Component<ScalarT, IdxT>::residual_indices_;

    public:
      using RealT = typename EMT::Component<ScalarT, IdxT>::RealT;
      using BusT  = EMT::Bus<ScalarT, IdxT>;
      using DataT = LoadRLData<RealT, IdxT>;

      static constexpr std::size_t own_variable_count = 3;
      static constexpr std::size_t own_equation_count = 3;
      static constexpr std::size_t port_count         = 1;
      static constexpr bool        is_affine          = true;

      static constexpr IdxT variable_count = static_cast<IdxT>(own_variable_count);

      static constexpr std::size_t port_width(std::size_t)
      {
        return 3;
      }

      static constexpr bool own_differential(std::size_t)
      {
        return true;
      }

      enum Var : std::size_t
      {
        I_A = 0,
        I_B = 1,
        I_C = 2,
        V_A = 3,
        V_B = 4,
        V_C = 5
      };

      enum Eq : std::size_t
      {
        EQ_A  = 0,
        EQ_B  = 1,
        EQ_C  = 2,
        INJ_A = 3,
        INJ_B = 4,
        INJ_C = 5
      };

      LoadRL(BusT* bus, const DataT& data);
      explicit LoadRL(const DataT& data);
      virtual ~LoadRL();

      int                               setGridKitComponentID(IdxT id) override final;
      int                               allocate() override final;
      int                               initialize() override final;
      int                               tagDifferentiable() override final;
      int                               evaluateResidual() override final;
      int                               evaluateJacobian() override final;
      int                               verify() const override final;
      const Model::VariableMonitorBase* getMonitor() const override final;

      template <class S>
      void residual(const S* y, const S* yp, S* f, RealT) const
      {
        for (std::size_t phase = 0; phase < 3; ++phase)
        {
          f[EQ_A + phase] =
              static_cast<S>(r_[phase]) * y[I_A + phase]
              + static_cast<S>(l_[phase]) * yp[I_A + phase]
              + y[V_A + phase];
          f[INJ_A + phase] = y[I_A + phase];
        }
      }

      BusT*                     connectedBus(std::size_t port) const;
      const DataT&              data() const;
      const PhaseVector<RealT>& r() const;
      const PhaseVector<RealT>& l() const;

    private:
      void loadParameters();

      BusT*              bus_{nullptr};
      DataT              data_{};
      PhaseVector<RealT> r_{};
      PhaseVector<RealT> l_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Components/LoadRL/LoadRLImpl.hpp>
