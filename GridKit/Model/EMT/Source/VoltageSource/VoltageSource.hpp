/**
 * @file VoltageSource.hpp
 * @brief Declaration of an ideal balanced EMT abc voltage source.
 */

#pragma once

#include <array>
#include <cstddef>

#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class VoltageSource : public Component<ScalarT, IdxT>
    {
      using Component<ScalarT, IdxT>::size_;
      using Component<ScalarT, IdxT>::nnz_;
      using Component<ScalarT, IdxT>::alpha_;
      using Component<ScalarT, IdxT>::time_;
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
      using DataT    = VoltageSourceData<RealT, IdxT>;
      using Terminal = typename Component<ScalarT, IdxT>::TerminalView;

      static constexpr size_t PHASE_COUNT       = 3;
      static constexpr size_t CURRENT_OFFSET    = 0;
      static constexpr size_t VOLTAGE_OFFSET    = PHASE_COUNT;
      static constexpr size_t DERIVATIVE_OFFSET = 2 * PHASE_COUNT;
      static constexpr size_t VARIABLE_COUNT    = 3 * PHASE_COUNT;

      VoltageSource();
      explicit VoltageSource(const DataT& data);

      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      auto           terminalCount() const -> size_t override final;
      int            bindTerminal(size_t terminal, Terminal view) override final;
      const ScalarT* terminalCurrent(size_t terminal) const override final;

      auto voltage(size_t phase) const -> ScalarT;

    private:
      auto jacobianEntryCapacity() const -> size_t;
      auto angularFrequency() const -> RealT;
      auto balancedVoltage(RealT t) const -> std::array<ScalarT, PHASE_COUNT>;
      auto balancedDerivative(RealT t) const -> std::array<ScalarT, PHASE_COUNT>;
      bool terminalIsBound(size_t terminal) const;

    private:
      DataT data_;

      Terminal terminal_{};
      bool     bound_{false};

      std::array<ScalarT, PHASE_COUNT> source_voltage_{};
      std::array<ScalarT, PHASE_COUNT> source_derivative_{};
      std::array<ScalarT, PHASE_COUNT> i_{};
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSourceImpl.hpp>
