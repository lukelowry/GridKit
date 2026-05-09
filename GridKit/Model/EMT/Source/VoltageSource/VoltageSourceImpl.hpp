/**
 * @file VoltageSourceImpl.hpp
 * @brief Implementation of an ideal balanced EMT abc voltage source.
 */

#pragma once

#include <cmath>

#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::VoltageSource()
    {
      size_ = static_cast<IdxT>(VARIABLE_COUNT);
    }

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::VoltageSource(const DataT& data)
      : data_(data)
    {
      size_ = static_cast<IdxT>(VARIABLE_COUNT);
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::allocate()
    {
      size_ = static_cast<IdxT>(VARIABLE_COUNT);
      this->allocateVectors();

      const auto max_nnz = jacobianEntryCapacity();
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[max_nnz];
        J_cols_buffer_ = new IdxT[max_nnz];
        J_vals_buffer_ = new RealT[max_nnz];
        J_.reserve(static_cast<IdxT>(max_nnz));
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::verify() const
    {
      int ret = 0;
      if (data_.amplitude <= 0.0)
      {
        Log::error() << "EMT::VoltageSource: amplitude must be positive\n";
        ret += 1;
      }
      if (data_.frequency <= 0.0)
      {
        Log::error() << "EMT::VoltageSource: frequency must be positive\n";
        ret += 1;
      }
      return ret;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::initialize()
    {
      source_voltage_    = balancedVoltage(time_);
      source_derivative_ = balancedDerivative(time_);
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        const auto current    = CURRENT_OFFSET + phase;
        const auto voltage    = VOLTAGE_OFFSET + phase;
        const auto derivative = DERIVATIVE_OFFSET + phase;

        y_[current]  = data_.current0[phase];
        yp_[current] = 0.0;
        i_[phase]    = y_[current];

        y_[voltage]  = source_voltage_[phase];
        yp_[voltage] = source_derivative_[phase];

        y_[derivative]  = source_derivative_[phase];
        yp_[derivative] = -angularFrequency() * angularFrequency() * source_voltage_[phase];
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::tagDifferentiable()
    {
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        tag_[phase] = false;
      }
      for (size_t state = VOLTAGE_OFFSET; state < VARIABLE_COUNT; ++state)
      {
        tag_[state] = true;
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateResidual()
    {
      if (!terminalIsBound(0))
      {
        Log::error() << "EMT::VoltageSource: terminal is not bound\n";
        return 1;
      }

      const RealT omega2 = angularFrequency() * angularFrequency();
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        const auto current    = CURRENT_OFFSET + phase;
        const auto voltage    = VOLTAGE_OFFSET + phase;
        const auto derivative = DERIVATIVE_OFFSET + phase;

        i_[phase]                 = y_[current];
        source_voltage_[phase]    = y_[voltage];
        source_derivative_[phase] = y_[derivative];

        f_[current]    = terminal_.y[phase] - y_[voltage];
        f_[voltage]    = yp_[voltage] - y_[derivative];
        f_[derivative] = yp_[derivative] + omega2 * y_[voltage];
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateJacobian()
    {
      J_.zeroMatrix();

      if (!terminalIsBound(0))
      {
        Log::error() << "EMT::VoltageSource: terminal is not bound\n";
        return 1;
      }

      IdxT nnz       = 0;
      auto add_entry = [&](IdxT row, IdxT col, RealT value)
      {
        J_rows_buffer_[nnz] = row;
        J_cols_buffer_[nnz] = col;
        J_vals_buffer_[nnz] = value;
        ++nnz;
      };

      const RealT omega2 = angularFrequency() * angularFrequency();
      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        const auto current    = CURRENT_OFFSET + phase;
        const auto voltage    = VOLTAGE_OFFSET + phase;
        const auto derivative = DERIVATIVE_OFFSET + phase;

        add_entry(terminal_.residual_index + static_cast<IdxT>(phase),
                  this->variableIndex(current),
                  -1.0);
        add_entry(this->residualIndex(current),
                  terminal_.variable_index + static_cast<IdxT>(phase),
                  1.0);
        add_entry(this->residualIndex(current),
                  this->variableIndex(voltage),
                  -1.0);
        add_entry(this->residualIndex(voltage),
                  this->variableIndex(voltage),
                  alpha_);
        add_entry(this->residualIndex(voltage),
                  this->variableIndex(derivative),
                  -1.0);
        add_entry(this->residualIndex(derivative),
                  this->variableIndex(voltage),
                  omega2);
        add_entry(this->residualIndex(derivative),
                  this->variableIndex(derivative),
                  alpha_);
      }

      J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);
      nnz_ = nnz;

      return 0;
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::terminalCount() const -> size_t
    {
      return 1;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::bindTerminal(size_t terminal, Terminal view)
    {
      if (terminal != 0 || view.y == nullptr || view.yp == nullptr)
      {
        return 1;
      }
      terminal_ = view;
      bound_    = true;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    const ScalarT* VoltageSource<ScalarT, IdxT>::terminalCurrent(size_t terminal) const
    {
      if (terminal != 0)
      {
        return nullptr;
      }
      return i_.data();
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::voltage(size_t phase) const -> ScalarT
    {
      return phase < PHASE_COUNT ? source_voltage_[phase] : ScalarT{0};
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::jacobianEntryCapacity() const -> size_t
    {
      return static_cast<size_t>(7) * PHASE_COUNT;
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::angularFrequency() const -> RealT
    {
      constexpr RealT pi = static_cast<RealT>(3.141592653589793238462643383279502884);
      return static_cast<RealT>(2.0) * pi * data_.frequency;
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::balancedVoltage(RealT t) const
        -> std::array<ScalarT, PHASE_COUNT>
    {
      constexpr RealT pi                = static_cast<RealT>(3.141592653589793238462643383279502884);
      constexpr RealT two_pi_over_three = static_cast<RealT>(2.0) * pi / static_cast<RealT>(3.0);

      const RealT theta = angularFrequency() * t + data_.phase;
      return {data_.amplitude * std::sin(theta),
              data_.amplitude * std::sin(theta - two_pi_over_three),
              data_.amplitude * std::sin(theta + two_pi_over_three)};
    }

    template <class ScalarT, typename IdxT>
    auto VoltageSource<ScalarT, IdxT>::balancedDerivative(RealT t) const
        -> std::array<ScalarT, PHASE_COUNT>
    {
      constexpr RealT pi                = static_cast<RealT>(3.141592653589793238462643383279502884);
      constexpr RealT two_pi_over_three = static_cast<RealT>(2.0) * pi / static_cast<RealT>(3.0);

      const RealT omega = angularFrequency();
      const RealT theta = omega * t + data_.phase;
      return {data_.amplitude * omega * std::cos(theta),
              data_.amplitude * omega * std::cos(theta - two_pi_over_three),
              data_.amplitude * omega * std::cos(theta + two_pi_over_three)};
    }

    template <class ScalarT, typename IdxT>
    bool VoltageSource<ScalarT, IdxT>::terminalIsBound(size_t terminal) const
    {
      return terminal == 0 && bound_;
    }

  } // namespace EMT
} // namespace GridKit
