#pragma once

#include <algorithm>
#include <utility>

#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::Bus()
      : Bus(INVALID_INDEX<IdxT>, std::vector<RealT>{0.0, 0.0, 0.0})
    {
    }

    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::Bus(std::vector<RealT> v0)
      : Bus(INVALID_INDEX<IdxT>, std::move(v0))
    {
    }

    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::Bus(IdxT bus_id, std::vector<RealT> v0)
      : v0_(std::move(v0))
    {
      bus_id_ = bus_id;
      size_   = static_cast<IdxT>(v0_.size());
    }

    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::~Bus() = default;

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::verify() const
    {
      if (v0_.empty())
      {
        Log::error() << "Bus: initial voltage vector must be non-empty\n";
        return 1;
      }
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::setBusID(IdxT bus_id)
    {
      bus_id_ = bus_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::allocate()
    {
      size_        = static_cast<IdxT>(v0_.size());
      const auto n = static_cast<std::size_t>(size_);

      y_.assign(n, ScalarT{0.0});
      yp_.assign(n, ScalarT{0.0});
      f_.assign(n, ScalarT{0.0});
      tag_.assign(n, false);
      abs_tol_.assign(n, ScalarT{0.0});
      variable_indices_.resize(n);
      residual_indices_.resize(n);

      for (IdxT j = 0; j < size_; ++j)
      {
        this->setVariableIndex(j, j);
        this->setResidualIndex(j, j);
      }

      nnz_ = 0;
      J_.zeroMatrix();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::initialize()
    {
      for (std::size_t n = 0; n < v0_.size(); ++n)
      {
        y_[n] = v0_[n];
      }
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
    {
      std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::evaluateResidual()
    {
      std::fill(f_.begin(), f_.end(), ScalarT{0.0});
      return 0;
    }

  } // namespace EMT
} // namespace GridKit
