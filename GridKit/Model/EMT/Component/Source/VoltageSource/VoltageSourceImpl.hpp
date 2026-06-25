#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    VoltageSource<scalar_type, index_type, N>::VoltageSource(BusT* bus)
      : VoltageSource(bus, ModelDataT{})
    {
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    VoltageSource<scalar_type, index_type, N>::VoltageSource(BusT* bus, const ModelDataT& data)
      : bus_(bus),
        data_(data)
    {
      size_  = 0;
      time_  = RealT{0.0};
      alpha_ = RealT{0.0};
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::setGridKitComponentID(IdxT gridkit_component_id)
    {
      gridkit_component_id_ = gridkit_component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::verify() const
    {
      int ret = 0;

      if (bus_ == nullptr)
      {
        ::GridKit::Utilities::Logger::error() << "VoltageSource: bus pointer is null\n";
        ++ret;
      }

      if (bus_ != nullptr && bus_->phaseCount() != static_cast<IdxT>(N))
      {
        ::GridKit::Utilities::Logger::error() << "VoltageSource: bus phase count does not match model dimension\n";
        ++ret;
      }

      if (!std::isfinite(data_.omega) || data_.omega <= RealT{0.0})
      {
        ::GridKit::Utilities::Logger::error() << "VoltageSource: omega must be finite and positive\n";
        ++ret;
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        if (!std::isfinite(data_.E[n]) || data_.E[n] < RealT{0.0})
        {
          ::GridKit::Utilities::Logger::error() << "VoltageSource: E must be finite and nonnegative\n";
          ++ret;
        }

        if (!std::isfinite(data_.phi[n]))
        {
          ::GridKit::Utilities::Logger::error() << "VoltageSource: phi contains a non-finite value\n";
          ++ret;
        }

        if (!std::isfinite(data_.G[n]) || data_.G[n] <= RealT{0.0})
        {
          ::GridKit::Utilities::Logger::error() << "VoltageSource: G must be finite and positive\n";
          ++ret;
        }
      }

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::allocate()
    {
      if (bus_ == nullptr || bus_->phaseCount() != static_cast<IdxT>(N))
      {
        return 1;
      }

      if (!allocated_)
      {
        allocateVectors(size_);
      }

      assert(y_.empty());
      assert(yp_.empty());
      assert(f_.empty());
      assert(tag_.empty());
      assert(abs_tol_.empty());

      variable_indices_.clear();
      residual_indices_.clear();
      wb_.assign(N, ScalarT{0.0});
      h_.assign(N, ScalarT{0.0});

      nnz_ = 0;
      J_.zeroMatrix();
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::initialize()
    {
      readBusVoltage();
      return evaluateBusResidual(y_.data(), yp_.data(), wb_.data(), h_.data());
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::tagDifferentiable()
    {
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::setAbsoluteTolerance([[maybe_unused]] RealT rel_tol)
    {
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void VoltageSource<scalar_type, index_type, N>::readBusVoltage()
    {
      for (std::size_t n = 0; n < N; ++n)
      {
        wb_[n] = bus_->V(static_cast<IdxT>(n));
      }
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    __attribute__((always_inline)) int VoltageSource<scalar_type, index_type, N>::evaluateBusResidual(
        [[maybe_unused]] ScalarT* y,
        [[maybe_unused]] ScalarT* yp,
        ScalarT*                  wb,
        ScalarT*                  h)
    {
      const RealT sqrt2 = std::sqrt(RealT{2.0});

      for (std::size_t n = 0; n < N; ++n)
      {
        const RealT v_src = sqrt2 * data_.E[n] * std::cos(data_.omega * time_ + data_.phi[n]);
        h[n]              = data_.G[n] * (v_src - wb[n]);
      }

      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::evaluateResidual()
    {
      readBusVoltage();
      const int ret = evaluateBusResidual(y_.data(), yp_.data(), wb_.data(), h_.data());
      this->addCurrentInjection(*bus_, h_.data());
      return ret;
    }

#ifndef GRIDKIT_ENABLE_ENZYME
    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::evaluateJacobian()
    {
      return 0;
    }
#endif
  } // namespace EMT
} // namespace GridKit
