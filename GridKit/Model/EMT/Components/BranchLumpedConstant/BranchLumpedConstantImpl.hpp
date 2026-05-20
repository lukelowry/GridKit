#pragma once

#include <algorithm>

#include <GridKit/Model/EMT/Components/BranchLumpedConstant/BranchLumpedConstant.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    BranchLumpedConstant<ScalarT, IdxT>::BranchLumpedConstant(BusT*        from_bus,
                                                              BusT*        to_bus,
                                                              const DataT& data)
      : from_bus_(from_bus),
        to_bus_(to_bus),
        data_(data)
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    BranchLumpedConstant<ScalarT, IdxT>::~BranchLumpedConstant()
    {
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::allocate()
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
    int BranchLumpedConstant<ScalarT, IdxT>::initialize()
    {
      std::fill(y_.begin(), y_.end(), ScalarT{0.0});
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::evaluateResidual()
    {
      std::fill(f_.begin(), f_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::verify() const
    {
      return (from_bus_ == nullptr ? 1 : 0) + (to_bus_ == nullptr ? 1 : 0);
    }
  } // namespace EMT
} // namespace GridKit
