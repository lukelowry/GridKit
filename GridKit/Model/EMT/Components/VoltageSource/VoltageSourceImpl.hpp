#pragma once

#include <algorithm>

#include <GridKit/Model/EMT/Components/VoltageSource/VoltageSource.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::VoltageSource(BusT* bus, const DataT& data)
      : bus_(bus),
        data_(data)
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::~VoltageSource()
    {
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::allocate()
    {
      size_ = variable_count;
      y_.clear();
      yp_.clear();
      f_.clear();
      tag_.clear();
      variable_indices_.clear();
      residual_indices_.clear();
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::initialize()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::tagDifferentiable()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateResidual()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::verify() const
    {
      return bus_ == nullptr ? 1 : 0;
    }
  } // namespace EMT
} // namespace GridKit
