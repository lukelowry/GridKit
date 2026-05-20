#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Components/LoadRL/LoadRLData.hpp>

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

      static constexpr IdxT variable_count = 3;

      LoadRL(BusT* bus, const DataT& data);
      virtual ~LoadRL();

      int setGridKitComponentID(IdxT id) override final;
      int allocate() override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;
      int verify() const override final;

    private:
      BusT* bus_{nullptr};
      DataT data_{};
    };
  } // namespace EMT
} // namespace GridKit
