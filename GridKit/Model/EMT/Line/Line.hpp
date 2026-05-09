/**
 * @file Line.hpp
 * @brief Declaration of an EMT line with terminal characteristic admittance.
 */

#pragma once

#include <array>
#include <cstddef>

#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Line/LineData.hpp>
#include <GridKit/Model/EMT/RationalApprox/RationalApprox.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class Line : public Component<ScalarT, IdxT>
    {
      using Component<ScalarT, IdxT>::size_;
      using Component<ScalarT, IdxT>::nnz_;
      using Component<ScalarT, IdxT>::alpha_;
      using Component<ScalarT, IdxT>::y_;
      using Component<ScalarT, IdxT>::yp_;
      using Component<ScalarT, IdxT>::tag_;
      using Component<ScalarT, IdxT>::f_;
      using Component<ScalarT, IdxT>::J_;
      using Component<ScalarT, IdxT>::J_rows_buffer_;
      using Component<ScalarT, IdxT>::J_cols_buffer_;
      using Component<ScalarT, IdxT>::J_vals_buffer_;

    public:
      using RealT    = typename Component<ScalarT, IdxT>::RealT;
      using DataT    = LineData<RealT, IdxT>;
      using Terminal = typename Component<ScalarT, IdxT>::TerminalView;

      static constexpr size_t PHASE_COUNT    = 3;
      static constexpr size_t TERMINAL_COUNT = 2;

      Line();
      explicit Line(const DataT& data);

      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      auto           terminalCount() const -> size_t override final;
      int            bindTerminal(size_t terminal, Terminal view) override final;
      const ScalarT* terminalCurrent(size_t terminal) const override final;
      bool           terminalHasDerivativeFeedthrough(size_t terminal) const override final;

      auto terminalStateCount() const -> size_t;
      auto stateCount() const -> size_t;

    private:
      auto jacobianEntryCapacity() const -> size_t;
      auto terminalStateOffset(size_t terminal) const -> size_t;

      auto terminalState(size_t terminal, size_t state = 0) -> ScalarT*;
      auto terminalState(size_t terminal, size_t state = 0) const -> const ScalarT*;
      auto terminalStateDerivative(size_t terminal, size_t state = 0) -> ScalarT*;
      auto terminalStateDerivative(size_t terminal, size_t state = 0) const -> const ScalarT*;

      auto terminalStateColumn(size_t terminal) const -> IdxT;
      auto terminalResidualRow(size_t terminal) const -> IdxT;
      bool terminalIsBound(size_t terminal) const;

    private:
      DataT                         data_;
      RationalApprox<ScalarT, IdxT> yc_;

      std::array<Terminal, TERMINAL_COUNT>                         terminals_{};
      std::array<bool, TERMINAL_COUNT>                             bound_{};
      std::array<std::array<ScalarT, PHASE_COUNT>, TERMINAL_COUNT> i_shunt_{};
      std::array<std::array<ScalarT, PHASE_COUNT>, TERMINAL_COUNT> i_{};
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Line/LineImpl.hpp>
