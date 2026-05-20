#pragma once

#include <algorithm>

#include <GridKit/Model/EMT/Components/Breaker/Breaker.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    Breaker<ScalarT, IdxT>::Breaker(BusT* from_bus, BusT* to_bus, const DataT& data)
      : from_bus_(from_bus),
        to_bus_(to_bus),
        data_(data)
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    Breaker<ScalarT, IdxT>::~Breaker()
    {
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::allocate()
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
    int Breaker<ScalarT, IdxT>::initialize()
    {
      std::fill(y_.begin(), y_.end(), ScalarT{0.0});
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), false);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::evaluateResidual()
    {
      // Residual assembly is system-driven via SystemModel + LocalMap; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::evaluateJacobian()
    {
      // Jacobian assembly is system-driven via SystemModel + SparseAD; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::verify() const
    {
      return (from_bus_ == nullptr ? 1 : 0) + (to_bus_ == nullptr ? 1 : 0);
    }
  } // namespace EMT
} // namespace GridKit
