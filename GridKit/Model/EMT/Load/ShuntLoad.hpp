/**
 * @file ShuntLoad.hpp
 * @brief Declaration of an EMT abc shunt conductance load.
 */

#pragma once

#include <array>
#include <cstddef>

#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Load/ShuntLoadData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class ShuntLoad : public Component<ScalarT, IdxT>
    {
      using Component<ScalarT, IdxT>::size_;
      using Component<ScalarT, IdxT>::nnz_;
      using Component<ScalarT, IdxT>::J_;
      using Component<ScalarT, IdxT>::J_rows_buffer_;
      using Component<ScalarT, IdxT>::J_cols_buffer_;
      using Component<ScalarT, IdxT>::J_vals_buffer_;

    public:
      using RealT    = typename Component<ScalarT, IdxT>::RealT;
      using DataT    = ShuntLoadData<RealT, IdxT>;
      using Terminal = typename Component<ScalarT, IdxT>::TerminalView;

      static constexpr size_t PHASE_COUNT = 3;

      ShuntLoad();
      explicit ShuntLoad(const DataT& data);

      int  allocate() override final;
      int  verify() const override final;
      int  initialize() override final;
      int  tagDifferentiable() override final;
      int  evaluateResidual() override final;
      int  evaluateJacobian() override final;
      void apply(Action action) override final;

      auto           terminalCount() const -> size_t override final;
      int            bindTerminal(size_t terminal, Terminal view) override final;
      const ScalarT* terminalCurrent(size_t terminal) const override final;

      void setStatus(bool status);
      auto status() const -> bool;
      void setClosed(bool closed);
      auto isClosed() const -> bool;

    private:
      auto matrixIndex(size_t row, size_t col) const -> size_t;
      auto jacobianEntryCapacity() const -> size_t;
      bool terminalIsBound(size_t terminal) const;

    private:
      DataT data_;

      Terminal terminal_{};
      bool     bound_{false};
      bool     closed_{true};

      std::array<ScalarT, PHASE_COUNT> i_{};
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Load/ShuntLoadImpl.hpp>
