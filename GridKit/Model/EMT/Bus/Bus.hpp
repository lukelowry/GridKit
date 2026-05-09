/**
 * @file Bus.hpp
 * @brief Declaration of a pure EMT abc network bus.
 */

#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/EMT/Component.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class Bus : public Component<ScalarT, IdxT>
    {
      using Component<ScalarT, IdxT>::size_;
      using Component<ScalarT, IdxT>::nnz_;
      using Component<ScalarT, IdxT>::y_;
      using Component<ScalarT, IdxT>::yp_;
      using Component<ScalarT, IdxT>::tag_;
      using Component<ScalarT, IdxT>::f_;
      using Component<ScalarT, IdxT>::J_;

    public:
      using RealT = typename Component<ScalarT, IdxT>::RealT;
      using DataT = BusData<RealT, IdxT>;

      static constexpr size_t PHASE_COUNT = 3;

      Bus();
      explicit Bus(const DataT& data);

      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      auto stateCount() const -> size_t;

      ScalarT&       va();
      ScalarT&       vb();
      ScalarT&       vc();
      const ScalarT& va() const;
      const ScalarT& vb() const;
      const ScalarT& vc() const;

    private:
      DataT data_;
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Bus/BusImpl.hpp>
