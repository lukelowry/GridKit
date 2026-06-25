#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFit.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    class LineLumped final : public EMT::Component<scalar_type, index_type>
    {
      using EMT::Component<scalar_type, index_type>::gridkit_component_id_;
      using EMT::Component<scalar_type, index_type>::size_;
      using EMT::Component<scalar_type, index_type>::nnz_;
      using EMT::Component<scalar_type, index_type>::y_;
      using EMT::Component<scalar_type, index_type>::yp_;
      using EMT::Component<scalar_type, index_type>::f_;
      using EMT::Component<scalar_type, index_type>::tag_;
      using EMT::Component<scalar_type, index_type>::abs_tol_;
      using EMT::Component<scalar_type, index_type>::J_;
      using EMT::Component<scalar_type, index_type>::variable_indices_;
      using EMT::Component<scalar_type, index_type>::residual_indices_;
      using EMT::Component<scalar_type, index_type>::alpha_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using BaseT      = EMT::Component<ScalarT, IdxT>;
      using RealT      = typename BaseT::RealT;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ModelDataT = LineLumpedData<RealT, IdxT, N>;
      using VectorFitT = EMT::VectorFit<ScalarT, IdxT, N, N>;
      using SignalT    = PhasorDynamics::SignalNode<ScalarT, IdxT>;

      static_assert(N > 0, "LineLumped phase count must be positive");

      LineLumped(BusT* bus1, BusT* bus2);
      LineLumped(BusT* bus1, BusT* bus2, const ModelDataT& data);
      ~LineLumped() override = default;

      int  setGridKitComponentID(IdxT) override final;
      int  verify() const override final;
      int  allocate() override final;
      int  initialize() override final;
      int  tagDifferentiable() override final;
      int  setAbsoluteTolerance(RealT rel_tol) override final;
      int  evaluateResidual() override final;
      int  evaluateJacobian() override final;
      void updateTime(RealT t, RealT a) override final;

      ScalarT& I(IdxT n)
      {
        return y_[static_cast<std::size_t>(n)];
      }

      const ScalarT& I(IdxT n) const
      {
        return y_[static_cast<std::size_t>(n)];
      }

    private:
      ScalarT& i(std::size_t n)
      {
        return y_[n];
      }

      const ScalarT& i(std::size_t n) const
      {
        return y_[n];
      }

      const ScalarT& v1(std::size_t n) const
      {
        return bus1_->V(static_cast<IdxT>(n));
      }

      const ScalarT& v2(std::size_t n) const
      {
        return bus2_->V(static_cast<IdxT>(n));
      }

      const ScalarT& seriesVoltageDrop(std::size_t n) const
      {
        return y_[series_z_output_ + n];
      }

      const ScalarT& shuntCurrent1(std::size_t n) const
      {
        return y_[shunt_y1_output_ + n];
      }

      const ScalarT& shuntCurrent2(std::size_t n) const
      {
        return y_[shunt_y2_output_ + n];
      }

      static typename ModelDataT::VectorFitDataT scaleVectorFitData(
          const typename ModelDataT::VectorFitDataT& data,
          RealT                                      scale);

      void setLayout();
      void wireSubmodelSignals();
      void copyParentIndicesToSubmodels();
      void copyParentStateToSubmodels();
      void copySubmodelStateToParent();
      void copySubmodelResiduals();
      int  evaluateSubmodelResiduals();
      void evaluateTerminalCurrents();

      BusT*      bus1_{nullptr};
      BusT*      bus2_{nullptr};
      ModelDataT data_{};

      VectorFitT series_z_;
      VectorFitT shunt_y1_;
      VectorFitT shunt_y2_;

      std::size_t series_z_first_{0};
      std::size_t shunt_y1_first_{0};
      std::size_t shunt_y2_first_{0};
      std::size_t series_z_output_{0};
      std::size_t shunt_y1_output_{0};
      std::size_t shunt_y2_output_{0};

      std::array<SignalT, N> i_nodes_{};
      std::array<SignalT, N> v1_nodes_{};
      std::array<SignalT, N> v2_nodes_{};
      std::array<SignalT, N> z_out_nodes_{};
      std::array<SignalT, N> y1_out_nodes_{};
      std::array<SignalT, N> y2_out_nodes_{};

      std::array<ScalarT, N> i_inj1_{};
      std::array<ScalarT, N> i_inj2_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedImpl.hpp>
#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedEnzyme.hpp>
#endif
