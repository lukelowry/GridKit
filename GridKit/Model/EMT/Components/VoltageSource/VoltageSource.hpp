#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Components/VoltageSource/VoltageSourceData.hpp>
#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class VoltageSource final : public EMT::Component<ScalarT, IdxT>
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
      using DataT = VoltageSourceData<RealT, IdxT>;

      static constexpr std::size_t own_variable_count = 0;
      static constexpr std::size_t own_equation_count = 0;
      static constexpr std::size_t port_count         = 1;
      static constexpr bool        is_affine          = true;

      static constexpr IdxT variable_count = 0;

      static constexpr std::size_t port_width(std::size_t)
      {
        return 3;
      }

      static constexpr bool own_differential(std::size_t)
      {
        return false;
      }

      enum Var : std::size_t
      {
        V_A = 0,
        V_B = 1,
        V_C = 2
      };

      enum Eq : std::size_t
      {
        INJ_A = 0,
        INJ_B = 1,
        INJ_C = 2
      };

      VoltageSource(BusT* bus, const DataT& data);
      explicit VoltageSource(const DataT& data);
      virtual ~VoltageSource();

      int                               setGridKitComponentID(IdxT id) override final;
      int                               allocate() override final;
      int                               initialize() override final;
      int                               tagDifferentiable() override final;
      int                               evaluateResidual() override final;
      int                               evaluateJacobian() override final;
      int                               verify() const override final;
      const Model::VariableMonitorBase* getMonitor() const override final;

      template <class S>
      void residual(const S* y, const S*, S* f, RealT t) const
      {
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        for (std::size_t phase = 0; phase < 3; ++phase)
        {
          const RealT e_inst = sqrt2 * e_[phase] * std::cos(omega0_ * t + phi_[phase]);
          f[INJ_A + phase] =
              (static_cast<S>(e_inst) - y[V_A + phase]) / static_cast<S>(r_[phase]);
        }
      }

      BusT*                     connectedBus(std::size_t port) const;
      const DataT&              data() const;
      const PhaseVector<RealT>& e() const;
      const PhaseVector<RealT>& phi() const;
      const PhaseVector<RealT>& r() const;
      RealT                     omega0() const;

    private:
      void loadParameters();

      BusT*              bus_{nullptr};
      DataT              data_{};
      PhaseVector<RealT> e_{};
      PhaseVector<RealT> phi_{};
      PhaseVector<RealT> r_{};
      RealT              omega0_{0.0};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Components/VoltageSource/VoltageSourceImpl.hpp>
