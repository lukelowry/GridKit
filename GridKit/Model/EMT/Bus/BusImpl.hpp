/**
 * @file BusImpl.hpp
 * @brief Implementation of a pure EMT abc network bus.
 */

#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::Bus()
    {
      size_ = static_cast<IdxT>(PHASE_COUNT);
    }

    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::Bus(const DataT& data)
      : data_(data)
    {
      size_ = static_cast<IdxT>(PHASE_COUNT);
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::allocate()
    {
      size_ = static_cast<IdxT>(PHASE_COUNT);
      this->allocateVectors();
      J_.zeroMatrix();
      nnz_ = 0;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::verify() const
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::initialize()
    {
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        y_[phase]  = data_.v0[phase];
        yp_[phase] = data_.vp0[phase];
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::tagDifferentiable()
    {
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        tag_[phase] = false;
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::evaluateResidual()
    {
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        f_[phase] = 0.0;
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::evaluateJacobian()
    {
      J_.zeroMatrix();
      nnz_ = 0;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    auto Bus<ScalarT, IdxT>::stateCount() const -> size_t
    {
      return static_cast<size_t>(size_);
    }

    template <class ScalarT, typename IdxT>
    ScalarT& Bus<ScalarT, IdxT>::va()
    {
      return y_[0];
    }

    template <class ScalarT, typename IdxT>
    ScalarT& Bus<ScalarT, IdxT>::vb()
    {
      return y_[1];
    }

    template <class ScalarT, typename IdxT>
    ScalarT& Bus<ScalarT, IdxT>::vc()
    {
      return y_[2];
    }

    template <class ScalarT, typename IdxT>
    const ScalarT& Bus<ScalarT, IdxT>::va() const
    {
      return y_[0];
    }

    template <class ScalarT, typename IdxT>
    const ScalarT& Bus<ScalarT, IdxT>::vb() const
    {
      return y_[1];
    }

    template <class ScalarT, typename IdxT>
    const ScalarT& Bus<ScalarT, IdxT>::vc() const
    {
      return y_[2];
    }

  } // namespace EMT
} // namespace GridKit
