/**
 * @file LineImpl.hpp
 * @brief Implementation of an EMT line with terminal characteristic admittance.
 */

#pragma once

#include <GridKit/Model/EMT/Line/Line.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <class ScalarT, typename IdxT>
    Line<ScalarT, IdxT>::Line()
    {
      size_ = 0;
    }

    template <class ScalarT, typename IdxT>
    Line<ScalarT, IdxT>::Line(const DataT& data)
      : data_(data),
        yc_(data.characteristic_admittance)
    {
      size_ = static_cast<IdxT>(stateCount());
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::allocate()
    {
      size_ = static_cast<IdxT>(stateCount());
      this->allocateVectors();

      const auto max_nnz = jacobianEntryCapacity();
      if (J_rows_buffer_ == nullptr && max_nnz > 0)
      {
        J_rows_buffer_ = new IdxT[max_nnz];
        J_cols_buffer_ = new IdxT[max_nnz];
        J_vals_buffer_ = new RealT[max_nnz];
        J_.reserve(static_cast<IdxT>(max_nnz));
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::verify() const
    {
      int ret = yc_.verify();
      if (yc_.dimension() != PHASE_COUNT)
      {
        Log::error() << "EMT::Line: characteristic admittance dimension must equal 3\n";
        ret += 1;
      }
      return ret;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::initialize()
    {
      for (size_t terminal = 0; terminal < TERMINAL_COUNT; ++terminal)
      {
        if (!terminalIsBound(terminal))
        {
          Log::error() << "EMT::Line: terminal " << terminal << " is not bound\n";
          return 1;
        }

        if (terminalStateCount() > 0)
        {
          yc_.initialize(terminals_[terminal].y,
                         terminals_[terminal].yp,
                         terminalState(terminal),
                         terminalStateDerivative(terminal));
        }
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::tagDifferentiable()
    {
      for (size_t state = 0; state < stateCount(); ++state)
      {
        tag_[state] = true;
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::evaluateResidual()
    {
      for (size_t terminal = 0; terminal < TERMINAL_COUNT; ++terminal)
      {
        if (!terminalIsBound(terminal))
        {
          Log::error() << "EMT::Line: terminal " << terminal << " is not bound\n";
          return 1;
        }

        yc_.evaluateOutput(terminals_[terminal].y,
                           terminals_[terminal].yp,
                           terminalState(terminal),
                           i_shunt_[terminal].data());

        if (terminalStateCount() > 0)
        {
          yc_.evaluateStateResidual(terminals_[terminal].y,
                                    terminalState(terminal),
                                    terminalStateDerivative(terminal),
                                    f_.data() + terminalStateOffset(terminal));
        }
      }

      for (size_t phase = 0; phase < PHASE_COUNT; ++phase)
      {
        i_[0][phase] = i_shunt_[0][phase] - i_shunt_[1][phase];
        i_[1][phase] = i_shunt_[1][phase] - i_shunt_[0][phase];
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::evaluateJacobian()
    {
      J_.zeroMatrix();

      if (J_rows_buffer_ == nullptr && jacobianEntryCapacity() > 0)
      {
        Log::error() << "EMT::Line: allocate must be called before evaluateJacobian\n";
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

      for (size_t terminal = 0; terminal < TERMINAL_COUNT; ++terminal)
      {
        if (!terminalIsBound(terminal))
        {
          Log::error() << "EMT::Line: terminal " << terminal << " is not bound\n";
          return 1;
        }

        const size_t remote = 1 - terminal;

        yc_.addOutputJacobianEntries(terminals_[terminal].residual_index,
                                     terminals_[terminal].variable_index,
                                     terminalStateColumn(terminal),
                                     alpha_,
                                     static_cast<RealT>(-1.0),
                                     add_entry);
        yc_.addOutputJacobianEntries(terminals_[terminal].residual_index,
                                     terminals_[remote].variable_index,
                                     terminalStateColumn(remote),
                                     alpha_,
                                     static_cast<RealT>(1.0),
                                     add_entry);

        if (terminalStateCount() > 0)
        {
          yc_.addStateJacobianEntries(terminalResidualRow(terminal),
                                      terminals_[terminal].variable_index,
                                      terminalStateColumn(terminal),
                                      alpha_,
                                      add_entry);
        }
      }

      J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);
      nnz_ = nnz;

      return 0;
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalCount() const -> size_t
    {
      return TERMINAL_COUNT;
    }

    template <class ScalarT, typename IdxT>
    int Line<ScalarT, IdxT>::bindTerminal(size_t terminal, Terminal view)
    {
      if (terminal >= TERMINAL_COUNT || view.y == nullptr || view.yp == nullptr)
      {
        return 1;
      }
      terminals_[terminal] = view;
      bound_[terminal]     = true;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    const ScalarT* Line<ScalarT, IdxT>::terminalCurrent(size_t terminal) const
    {
      if (terminal >= TERMINAL_COUNT)
      {
        return nullptr;
      }
      return i_[terminal].data();
    }

    template <class ScalarT, typename IdxT>
    bool Line<ScalarT, IdxT>::terminalHasDerivativeFeedthrough(size_t terminal) const
    {
      return terminal < TERMINAL_COUNT && yc_.hasDerivativeFeedthrough();
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalStateCount() const -> size_t
    {
      return yc_.stateCount();
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::stateCount() const -> size_t
    {
      return TERMINAL_COUNT * terminalStateCount();
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::jacobianEntryCapacity() const -> size_t
    {
      return TERMINAL_COUNT * (static_cast<size_t>(2) * yc_.outputJacobianEntryCount() + yc_.stateJacobianEntryCount());
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalStateOffset(size_t terminal) const -> size_t
    {
      return terminal * terminalStateCount();
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalState(size_t terminal, size_t state) -> ScalarT*
    {
      return y_.data() + terminalStateOffset(terminal) + state;
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalState(size_t terminal, size_t state) const -> const ScalarT*
    {
      return y_.data() + terminalStateOffset(terminal) + state;
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalStateDerivative(size_t terminal, size_t state) -> ScalarT*
    {
      return yp_.data() + terminalStateOffset(terminal) + state;
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalStateDerivative(size_t terminal, size_t state) const
        -> const ScalarT*
    {
      return yp_.data() + terminalStateOffset(terminal) + state;
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalStateColumn(size_t terminal) const -> IdxT
    {
      return this->variableRange().begin + static_cast<IdxT>(terminalStateOffset(terminal));
    }

    template <class ScalarT, typename IdxT>
    auto Line<ScalarT, IdxT>::terminalResidualRow(size_t terminal) const -> IdxT
    {
      return this->residualRange().begin + static_cast<IdxT>(terminalStateOffset(terminal));
    }

    template <class ScalarT, typename IdxT>
    bool Line<ScalarT, IdxT>::terminalIsBound(size_t terminal) const
    {
      return terminal < TERMINAL_COUNT && bound_[terminal];
    }

  } // namespace EMT
} // namespace GridKit
