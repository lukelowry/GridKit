#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Components/BranchLumpedConstant/BranchLumpedConstantData.hpp>
#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class BranchLumpedConstant final : public EMT::Component<ScalarT, IdxT>
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
      using DataT = BranchLumpedConstantData<RealT, IdxT>;

      static constexpr std::size_t own_variable_count = 3;
      static constexpr std::size_t own_equation_count = 3;
      static constexpr std::size_t port_count         = 2;
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
        I_A  = 0,
        I_B  = 1,
        I_C  = 2,
        VF_A = 3,
        VF_B = 4,
        VF_C = 5,
        VT_A = 6,
        VT_B = 7,
        VT_C = 8
      };

      enum Eq : std::size_t
      {
        EQ_A = 0,
        EQ_B = 1,
        EQ_C = 2,
        IF_A = 3,
        IF_B = 4,
        IF_C = 5,
        IT_A = 6,
        IT_B = 7,
        IT_C = 8
      };

      BranchLumpedConstant(BusT* from_bus, BusT* to_bus, const DataT& data);
      explicit BranchLumpedConstant(const DataT& data);
      virtual ~BranchLumpedConstant();

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
        for (std::size_t row = 0; row < 3; ++row)
        {
          S r_i{};
          S l_ip{};
          S g_from{};
          S c_from{};
          S g_to{};
          S c_to{};

          for (std::size_t col = 0; col < 3; ++col)
          {
            r_i    += static_cast<S>(R_[row][col]) * y[I_A + col];
            l_ip   += static_cast<S>(L_[row][col]) * yp[I_A + col];
            g_from += static_cast<S>(G_half_[row][col]) * y[VF_A + col];
            c_from += static_cast<S>(C_half_[row][col]) * yp[VF_A + col];
            g_to   += static_cast<S>(G_half_[row][col]) * y[VT_A + col];
            c_to   += static_cast<S>(C_half_[row][col]) * yp[VT_A + col];
          }

          f[EQ_A + row] = r_i + l_ip + y[VT_A + row] - y[VF_A + row];
          f[IF_A + row] = -g_from - c_from - y[I_A + row];
          f[IT_A + row] = -g_to - c_to + y[I_A + row];
        }
      }

      BusT*                     connectedBus(std::size_t port) const;
      const DataT&              data() const;
      const PhaseMatrix<RealT>& R() const;
      const PhaseMatrix<RealT>& L() const;
      const PhaseMatrix<RealT>& GHalf() const;
      const PhaseMatrix<RealT>& CHalf() const;

    private:
      void loadParameters();

      BusT*              from_bus_{nullptr};
      BusT*              to_bus_{nullptr};
      DataT              data_{};
      PhaseMatrix<RealT> r_per_m_{};
      PhaseMatrix<RealT> l_per_m_{};
      PhaseMatrix<RealT> g_per_m_{};
      PhaseMatrix<RealT> c_per_m_{};
      RealT              length_{0.0};
      PhaseMatrix<RealT> R_{};
      PhaseMatrix<RealT> L_{};
      PhaseMatrix<RealT> G_half_{};
      PhaseMatrix<RealT> C_half_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Components/BranchLumpedConstant/BranchLumpedConstantImpl.hpp>
