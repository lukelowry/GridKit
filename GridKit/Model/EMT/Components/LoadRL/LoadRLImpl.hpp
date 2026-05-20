#pragma once

#include <algorithm>

#include <GridKit/Model/EMT/Components/LoadRL/LoadRL.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    LoadRL<ScalarT, IdxT>::LoadRL(BusT* bus, const DataT& data)
      : bus_(bus),
        data_(data)
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    LoadRL<ScalarT, IdxT>::~LoadRL()
    {
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::allocate()
    {
      size_             = variable_count;
      const size_t size = static_cast<size_t>(size_);
      y_.resize(size);
      yp_.resize(size);
      f_.resize(size);
      tag_.resize(size);
      variable_indices_.resize(size);
      residual_indices_.resize(size);

      for (IdxT j = 0; j < size_; ++j)
      {
        this->setVariableIndex(j, j);
        this->setResidualIndex(j, j);
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::initialize()
    {
      std::fill(y_.begin(), y_.end(), ScalarT{0.0});
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::evaluateResidual()
    {
      std::fill(f_.begin(), f_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::verify() const
    {
      return bus_ == nullptr ? 1 : 0;
    }
  } // namespace EMT
} // namespace GridKit
