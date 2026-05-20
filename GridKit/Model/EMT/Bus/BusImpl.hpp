#pragma once

#include <algorithm>
#include <cmath>

#include <GridKit/Model/EMT/Bus/Bus.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::Bus()
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::Bus(const DataT& data)
      : data_(data)
    {
      size_ = variable_count;
    }

    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::Bus(RealT vm, RealT va)
    {
      data_.vm = vm;
      data_.va = va;
      size_    = variable_count;
    }

    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>::~Bus()
    {
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::setBusID(IdxT id)
    {
      data_.bus_id = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    IdxT Bus<ScalarT, IdxT>::busID() const
    {
      return data_.bus_id;
    }

    template <class ScalarT, typename IdxT>
    const std::string& Bus<ScalarT, IdxT>::name() const
    {
      return data_.name;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::allocate()
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
    int Bus<ScalarT, IdxT>::initialize()
    {
      const RealT pi        = std::acos(RealT{-1.0});
      const RealT shift     = RealT{2.0} * pi / RealT{3.0};
      const RealT sqrt2     = std::sqrt(RealT{2.0});
      const RealT freq      = data_.freq_base.value_or(RealT{60.0});
      const RealT omega     = RealT{2.0} * pi * freq;
      const RealT angles[3] = {data_.va, data_.va - shift, data_.va + shift};

      for (IdxT phase = 0; phase < size_; ++phase)
      {
        const RealT angle               = angles[static_cast<size_t>(phase)];
        y_[static_cast<size_t>(phase)]  = sqrt2 * data_.vm * std::cos(angle);
        yp_[static_cast<size_t>(phase)] = -sqrt2 * data_.vm * omega * std::sin(angle);
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::evaluateResidual()
    {
      std::fill(f_.begin(), f_.end(), ScalarT{0.0});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::verify() const
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    bool Bus<ScalarT, IdxT>::hasJacobian()
    {
      return true;
    }

    template <class ScalarT, typename IdxT>
    void Bus<ScalarT, IdxT>::updateTime(RealT, RealT)
    {
    }

    template <class ScalarT, typename IdxT>
    ScalarT& Bus<ScalarT, IdxT>::voltage(IdxT phase)
    {
      return y_.at(static_cast<size_t>(phase));
    }

    template <class ScalarT, typename IdxT>
    const ScalarT& Bus<ScalarT, IdxT>::voltage(IdxT phase) const
    {
      return y_.at(static_cast<size_t>(phase));
    }

    template <class ScalarT, typename IdxT>
    ScalarT& Bus<ScalarT, IdxT>::residualCurrent(IdxT phase)
    {
      return f_.at(static_cast<size_t>(phase));
    }

    template <class ScalarT, typename IdxT>
    const ScalarT& Bus<ScalarT, IdxT>::residualCurrent(IdxT phase) const
    {
      return f_.at(static_cast<size_t>(phase));
    }
  } // namespace EMT
} // namespace GridKit
