#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    class VoltageSource final : public EMT::Component<scalar_type, index_type>
    {
      using EMT::Component<scalar_type, index_type>::gridkit_component_id_;
      using EMT::Component<scalar_type, index_type>::size_;
      using EMT::Component<scalar_type, index_type>::nnz_;
      using EMT::Component<scalar_type, index_type>::y_;
      using EMT::Component<scalar_type, index_type>::yp_;
      using EMT::Component<scalar_type, index_type>::f_;
      using EMT::Component<scalar_type, index_type>::tag_;
      using EMT::Component<scalar_type, index_type>::abs_tol_;
      using EMT::Component<scalar_type, index_type>::wb_;
      using EMT::Component<scalar_type, index_type>::h_;
      using EMT::Component<scalar_type, index_type>::J_;
      using EMT::Component<scalar_type, index_type>::J_rows_buffer_;
      using EMT::Component<scalar_type, index_type>::J_cols_buffer_;
      using EMT::Component<scalar_type, index_type>::J_vals_buffer_;
      using EMT::Component<scalar_type, index_type>::variable_indices_;
      using EMT::Component<scalar_type, index_type>::residual_indices_;
      using EMT::Component<scalar_type, index_type>::time_;
      using EMT::Component<scalar_type, index_type>::alpha_;
      using EMT::Component<scalar_type, index_type>::offset_;
      using EMT::Component<scalar_type, index_type>::allocated_;
      using EMT::Component<scalar_type, index_type>::allocateVectors;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using BaseT      = EMT::Component<ScalarT, IdxT>;
      using RealT      = typename BaseT::RealT;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ModelDataT = VoltageSourceData<RealT, IdxT, N>;

      static_assert(N > 0, "VoltageSource phase count must be positive");

      explicit VoltageSource(BusT* bus);
      VoltageSource(BusT* bus, const ModelDataT& data);
      ~VoltageSource() override = default;

      int setGridKitComponentID(IdxT) override final;
      int verify() const override final;
      int allocate() override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT rel_tol) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      __attribute__((always_inline)) int evaluateBusResidual(
          ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* h);

    private:
      void readBusVoltage();

      BusT*      bus_{nullptr};
      ModelDataT data_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceImpl.hpp>
#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceEnzyme.hpp>
#endif
