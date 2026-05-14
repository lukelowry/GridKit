#pragma once

#include <span>
#include <stdexcept>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Assembly.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class PatternView
    {
    public:
      using Entry = typename Assembly<RealT, IdxT>::Entry;

      PatternView(std::vector<Entry>&   entries,
                  const Layout<IdxT>&   layout,
                  ComponentLayout<IdxT> component,
                  std::span<const IdxT> terminal_bus_ids,
                  std::span<const IdxT> input_variable_ids,
                  std::vector<bool>*    differential_variables = nullptr)
        : entries_{entries},
          layout_{layout},
          component_{component},
          terminal_bus_ids_{terminal_bus_ids},
          input_variable_ids_{input_variable_ids},
          differential_variables_{differential_variables}
      {
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

      IdxT inputColumn(IdxT input) const
      {
        const IdxT global = input_variable_ids_[input];
        if (global == INVALID_INDEX<IdxT>)
        {
          throw std::logic_error("EMT input is not connected");
        }
        return global;
      }

      void add(IdxT row, IdxT col)
      {
        entries_.emplace_back(row, col);
      }

      void markDifferential(IdxT column)
      {
        if (differential_variables_ != nullptr)
        {
          (*differential_variables_)[column] = true;
        }
      }

      void addEquationVariable(IdxT row, IdxT variable)
      {
        add(equationRow(row), variableColumn(variable));
      }

      void addEquationDerivative(IdxT row, IdxT variable)
      {
        markDifferential(variableColumn(variable));
        addEquationVariable(row, variable);
      }

      void addEquationTerminalVoltage(IdxT row, IdxT terminal, IdxT phase)
      {
        add(equationRow(row), terminalVoltageColumn(terminal, phase));
      }

      void addEquationTerminalVoltageDerivative(IdxT row, IdxT terminal, IdxT phase)
      {
        markDifferential(terminalVoltageColumn(terminal, phase));
        addEquationTerminalVoltage(row, terminal, phase);
      }

      void addEquationInput(IdxT row, IdxT input)
      {
        add(equationRow(row), inputColumn(input));
      }

      void addInjectionVariable(IdxT terminal, IdxT phase, IdxT variable)
      {
        add(terminalRow(terminal, phase), variableColumn(variable));
      }

      void addInjectionVariableDerivative(IdxT terminal, IdxT phase, IdxT variable)
      {
        markDifferential(variableColumn(variable));
        addInjectionVariable(terminal, phase, variable);
      }

      void addInjectionTerminalVoltage(IdxT row_terminal,
                                       IdxT row_phase,
                                       IdxT column_terminal,
                                       IdxT column_phase)
      {
        add(terminalRow(row_terminal, row_phase),
            terminalVoltageColumn(column_terminal, column_phase));
      }

      void addInjectionTerminalVoltageDerivative(IdxT row_terminal,
                                                 IdxT row_phase,
                                                 IdxT column_terminal,
                                                 IdxT column_phase)
      {
        markDifferential(terminalVoltageColumn(column_terminal, column_phase));
        addInjectionTerminalVoltage(row_terminal, row_phase, column_terminal, column_phase);
      }

      void addInjectionInput(IdxT terminal, IdxT phase, IdxT input)
      {
        add(terminalRow(terminal, phase), inputColumn(input));
      }

    private:
      std::vector<Entry>&   entries_;
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_bus_ids_;
      std::span<const IdxT> input_variable_ids_;
      std::vector<bool>*    differential_variables_{nullptr};
    };
  } // namespace EMT
} // namespace GridKit
