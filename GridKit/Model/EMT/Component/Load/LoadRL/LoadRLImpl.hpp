#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRL.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    LoadRL<scalar_type, index_type>::LoadRL(BusT* bus)
      : LoadRL(bus, ModelDataT{})
    {
    }

    template <typename scalar_type, typename index_type>
    LoadRL<scalar_type, index_type>::LoadRL(BusT* bus, const ModelDataT& data)
      : bus_(bus),
        data_(data)
    {
      size_  = 3;
      time_  = RealT{0.0};
      alpha_ = RealT{0.0};
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::setGridKitComponentID(IdxT gridkit_component_id)
    {
      gridkit_component_id_ = gridkit_component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::verify() const
    {
      int ret = 0;

      if (bus_ == nullptr)
      {
        ::GridKit::Utilities::Logger::error() << "LoadRL: bus pointer is null\n";
        ++ret;
      }

      if (bus_ != nullptr && bus_->phaseCount() != 3)
      {
        ::GridKit::Utilities::Logger::error() << "LoadRL: bus phase count must be 3\n";
        ++ret;
      }

      for (std::size_t n = 0; n < 3; ++n)
      {
        if (!std::isfinite(data_.R[n]) || data_.R[n] < RealT{0.0})
        {
          ::GridKit::Utilities::Logger::error() << "LoadRL: R must be finite and nonnegative\n";
          ++ret;
        }

        if (!std::isfinite(data_.L[n]) || data_.L[n] <= RealT{0.0})
        {
          ::GridKit::Utilities::Logger::error() << "LoadRL: L must be finite and positive\n";
          ++ret;
        }
      }

      if (!std::isfinite(data_.Iinj) || data_.Iinj < RealT{0.0})
      {
        ::GridKit::Utilities::Logger::error() << "LoadRL: Iinj must be finite and nonnegative\n";
        ++ret;
      }

      if (!std::isfinite(data_.theta))
      {
        ::GridKit::Utilities::Logger::error() << "LoadRL: theta must be finite\n";
        ++ret;
      }

      return ret;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::allocate()
    {
      if (bus_ == nullptr || bus_->phaseCount() != 3)
      {
        return 1;
      }

      const auto size = static_cast<std::size_t>(size_);
      if (!allocated_)
      {
        allocateVectors(size_);
      }

      assert(y_.size() == size);
      assert(yp_.size() == size);
      assert(f_.size() == size);
      assert(tag_.size() == size);
      assert(abs_tol_.size() == size);

      variable_indices_.resize(size);
      residual_indices_.resize(size);

      for (IdxT j = 0; j < size_; ++j)
      {
        variable_indices_[static_cast<std::size_t>(j)] = offset_ + j;
        residual_indices_[static_cast<std::size_t>(j)] = offset_ + j;
      }

      wb_.assign(size, ScalarT{0.0});
      h_.assign(size, ScalarT{0.0});

      nnz_ = 0;
      J_.zeroMatrix();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::initialize()
    {
      const RealT sqrt2 = std::sqrt(RealT{2.0});
      const RealT pi    = std::acos(RealT{-1.0});

      y_[0]  = sqrt2 * data_.Iinj * std::cos(data_.theta);
      y_[1]  = sqrt2 * data_.Iinj * std::cos(data_.theta - RealT{2.0} * pi / RealT{3.0});
      y_[2]  = sqrt2 * data_.Iinj * std::cos(data_.theta + RealT{2.0} * pi / RealT{3.0});
      yp_[0] = ScalarT{0.0};
      yp_[1] = ScalarT{0.0};
      yp_[2] = ScalarT{0.0};

      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::tagDifferentiable()
    {
      for (std::size_t n = 0; n < tag_.size(); ++n)
      {
        tag_[n] = ScalarT{1.0};
      }
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
    {
      for (std::size_t n = 0; n < abs_tol_.size(); ++n)
      {
        abs_tol_[n] = rel_tol;
      }
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void LoadRL<scalar_type, index_type>::readBusVoltage()
    {
      for (std::size_t n = 0; n < 3; ++n)
      {
        wb_[n] = bus_->V(static_cast<IdxT>(n));
      }
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) int LoadRL<scalar_type, index_type>::evaluateInternalResidual(
        ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* f)
    {
      for (std::size_t n = 0; n < 3; ++n)
      {
        f[n] = data_.R[n] * y[n] + data_.L[n] * yp[n] + wb[n];
      }

      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) int LoadRL<scalar_type, index_type>::evaluateBusResidual(
        ScalarT*                  y,
        [[maybe_unused]] ScalarT* yp,
        [[maybe_unused]] ScalarT* wb,
        ScalarT*                  h)
    {
      for (std::size_t n = 0; n < 3; ++n)
      {
        h[n] = y[n];
      }

      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::evaluateResidual()
    {
      readBusVoltage();
      int ret  = 0;
      ret     += evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), f_.data());
      ret     += evaluateBusResidual(y_.data(), yp_.data(), wb_.data(), h_.data());
      this->addCurrentInjection(*bus_, h_.data());
      return ret;
    }

#ifndef GRIDKIT_ENABLE_ENZYME
    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }
#endif
  } // namespace EMT
} // namespace GridKit
