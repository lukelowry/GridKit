/**
 * @file ShuntLoadImpl.hpp
 * @brief Implementation of an EMT abc shunt conductance load.
 */

#pragma once

#include <GridKit/Model/EMT/Load/ShuntLoad.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <class ScalarT, typename IdxT>
    ShuntLoad<ScalarT, IdxT>::ShuntLoad()
    {
      size_ = 0;
    }

    template <class ScalarT, typename IdxT>
    ShuntLoad<ScalarT, IdxT>::ShuntLoad(const DataT& data)
      : data_(data),
        closed_(data.closed)
    {
      size_ = 0;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::allocate()
    {
      size_ = 0;
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
    int ShuntLoad<ScalarT, IdxT>::verify() const
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::initialize()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::tagDifferentiable()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::evaluateResidual()
    {
      if (!terminalIsBound(0))
      {
        Log::error() << "EMT::ShuntLoad: terminal is not bound\n";
        return 1;
      }

      const RealT scale = closed_ ? static_cast<RealT>(1.0) : static_cast<RealT>(0.0);

      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        i_[phase] = 0.0;
        for (size_t col = 0; col < PHASE_COUNT; ++col)
        {
          i_[phase] += scale * data_.conductance[matrixIndex(phase, col)] * terminal_.y[col];
        }
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::evaluateJacobian()
    {
      J_.zeroMatrix();

      if (!terminalIsBound(0))
      {
        Log::error() << "EMT::ShuntLoad: terminal is not bound\n";
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

      const RealT scale = closed_ ? static_cast<RealT>(1.0) : static_cast<RealT>(0.0);

      for (size_t row = 0; row < PHASE_COUNT; ++row)
      {
        for (size_t col = 0; col < PHASE_COUNT; ++col)
        {
          add_entry(terminal_.residual_index + static_cast<IdxT>(row),
                    terminal_.variable_index + static_cast<IdxT>(col),
                    -scale * data_.conductance[matrixIndex(row, col)]);
        }
      }

      J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);
      nnz_ = nnz;

      return 0;
    }

    template <class ScalarT, typename IdxT>
    void ShuntLoad<ScalarT, IdxT>::apply(Action action)
    {
      setStatus(action == Action::On);
    }

    template <class ScalarT, typename IdxT>
    auto ShuntLoad<ScalarT, IdxT>::terminalCount() const -> size_t
    {
      return 1;
    }

    template <class ScalarT, typename IdxT>
    int ShuntLoad<ScalarT, IdxT>::bindTerminal(size_t terminal, Terminal view)
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
    const ScalarT* ShuntLoad<ScalarT, IdxT>::terminalCurrent(size_t terminal) const
    {
      if (terminal != 0)
      {
        return nullptr;
      }
      return i_.data();
    }

    template <class ScalarT, typename IdxT>
    void ShuntLoad<ScalarT, IdxT>::setStatus(bool status)
    {
      setClosed(status);
    }

    template <class ScalarT, typename IdxT>
    auto ShuntLoad<ScalarT, IdxT>::status() const -> bool
    {
      return isClosed();
    }

    template <class ScalarT, typename IdxT>
    void ShuntLoad<ScalarT, IdxT>::setClosed(bool closed)
    {
      closed_ = closed;
    }

    template <class ScalarT, typename IdxT>
    auto ShuntLoad<ScalarT, IdxT>::isClosed() const -> bool
    {
      return closed_;
    }

    template <class ScalarT, typename IdxT>
    auto ShuntLoad<ScalarT, IdxT>::matrixIndex(size_t row, size_t col) const -> size_t
    {
      return row * PHASE_COUNT + col;
    }

    template <class ScalarT, typename IdxT>
    auto ShuntLoad<ScalarT, IdxT>::jacobianEntryCapacity() const -> size_t
    {
      return PHASE_COUNT * PHASE_COUNT;
    }

    template <class ScalarT, typename IdxT>
    bool ShuntLoad<ScalarT, IdxT>::terminalIsBound(size_t terminal) const
    {
      return terminal == 0 && bound_;
    }

  } // namespace EMT
} // namespace GridKit
