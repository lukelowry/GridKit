#pragma once

#include <algorithm>
#include <span>
#include <stdexcept>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class JacobianView
    {
    public:
      JacobianView(RealT*                values,
                   const IdxT*           row_ptrs,
                   const IdxT*           cols,
                   const Layout<IdxT>&   layout,
                   ComponentLayout<IdxT> component,
                   std::span<const IdxT> terminal_bus_ids,
                   std::span<const IdxT> input_variable_ids,
                   RealT                 alpha)
        : values_{values},
          row_ptrs_{row_ptrs},
          cols_{cols},
          layout_{layout},
          component_{component},
          terminal_bus_ids_{terminal_bus_ids},
          input_variable_ids_{input_variable_ids},
          alpha_{alpha}
      {
      }

      RealT alpha() const
      {
        return alpha_;
      }

      IdxT equationRow(IdxT local) const
      {
        return component_.equation_offset + local;
      }

      IdxT variableColumn(IdxT local) const
      {
        return component_.variable_offset + local;
      }

      IdxT terminalRow(IdxT terminal, IdxT phase) const
      {
        return layout_.busEquation(terminal_bus_ids_[terminal], phase);
      }

      IdxT terminalVoltageColumn(IdxT terminal, IdxT phase) const
      {
        return layout_.busVariable(terminal_bus_ids_[terminal], phase);
      }

      IdxT inputColumn(IdxT port) const
      {
        const IdxT global = input_variable_ids_[port];
        if (global == INVALID_INDEX<IdxT>)
        {
          throw std::logic_error("EMT input is not connected");
        }
        return global;
      }

      void add(IdxT row, IdxT col, RealT value)
      {
        const IdxT  begin = row_ptrs_[row];
        const IdxT  end   = row_ptrs_[row + 1];
        const IdxT* it    = std::lower_bound(cols_ + begin, cols_ + end, col);
        if (it == cols_ + end || *it != col)
        {
          throw std::logic_error("EMT Jacobian entry is not present in the assembly pattern");
        }
        values_[it - cols_] += value;
      }

      void addEquationVariable(IdxT row, IdxT variable, RealT value)
      {
        add(equationRow(row), variableColumn(variable), value);
      }

      void addEquationDerivative(IdxT row, IdxT variable, RealT value)
      {
        add(equationRow(row), variableColumn(variable), alpha_ * value);
      }

      void addEquationTerminalVoltage(IdxT row, IdxT terminal, IdxT phase, RealT value)
      {
        add(equationRow(row), terminalVoltageColumn(terminal, phase), value);
      }

      void addEquationTerminalVoltageDerivative(IdxT row, IdxT terminal, IdxT phase, RealT value)
      {
        addEquationTerminalVoltage(row, terminal, phase, alpha_ * value);
      }

      void addEquationInput(IdxT row, IdxT port, RealT value)
      {
        add(equationRow(row), inputColumn(port), value);
      }

      void addInjectionVariable(IdxT terminal, IdxT phase, IdxT variable, RealT value)
      {
        add(terminalRow(terminal, phase), variableColumn(variable), value);
      }

      void addInjectionVariableDerivative(IdxT terminal, IdxT phase, IdxT variable, RealT value)
      {
        addInjectionVariable(terminal, phase, variable, alpha_ * value);
      }

      void addInjectionTerminalVoltage(IdxT  row_terminal,
                                       IdxT  row_phase,
                                       IdxT  column_terminal,
                                       IdxT  column_phase,
                                       RealT value)
      {
        add(terminalRow(row_terminal, row_phase),
            terminalVoltageColumn(column_terminal, column_phase),
            value);
      }

      void addInjectionTerminalVoltageDerivative(IdxT  row_terminal,
                                                 IdxT  row_phase,
                                                 IdxT  column_terminal,
                                                 IdxT  column_phase,
                                                 RealT value)
      {
        addInjectionTerminalVoltage(row_terminal,
                                    row_phase,
                                    column_terminal,
                                    column_phase,
                                    alpha_ * value);
      }

      void addInjectionInput(IdxT terminal, IdxT phase, IdxT port, RealT value)
      {
        add(terminalRow(terminal, phase), inputColumn(port), value);
      }

    private:
      RealT*                values_{nullptr};
      const IdxT*           row_ptrs_{nullptr};
      const IdxT*           cols_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_bus_ids_;
      std::span<const IdxT> input_variable_ids_;
      RealT                 alpha_{0.0};
    };
  } // namespace EMT
} // namespace GridKit
