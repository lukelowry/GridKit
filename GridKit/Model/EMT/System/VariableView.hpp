#pragma once

#include <array>
#include <span>
#include <stdexcept>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class VariableView
    {
    public:
      VariableView(const ScalarT*        y,
                   const ScalarT*        yp,
                   const Layout<IdxT>&   layout,
                   ComponentLayout<IdxT> component,
                   std::span<const IdxT> terminal_bus_ids,
                   std::span<const IdxT> input_variable_ids)
        : y_{y},
          yp_{yp},
          layout_{layout},
          component_{component},
          terminal_bus_ids_{terminal_bus_ids},
          input_variable_ids_{input_variable_ids}
      {
      }

      ScalarT variable(IdxT local) const
      {
        return y_[component_.variable_offset + local];
      }

      ScalarT derivative(IdxT local) const
      {
        return yp_[component_.variable_offset + local];
      }

      template <size_t N>
      std::array<ScalarT, N> variables(IdxT first = 0) const
      {
        std::array<ScalarT, N> values{};
        for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
        {
          values[i] = variable(first + i);
        }
        return values;
      }

      template <size_t N>
      std::array<ScalarT, N> derivatives(IdxT first = 0) const
      {
        std::array<ScalarT, N> values{};
        for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
        {
          values[i] = derivative(first + i);
        }
        return values;
      }

      std::array<ScalarT, 3> voltage(IdxT terminal) const
      {
        const IdxT bus = terminal_bus_ids_[terminal];
        return {y_[layout_.busVariable(bus, 0)],
                y_[layout_.busVariable(bus, 1)],
                y_[layout_.busVariable(bus, 2)]};
      }

      std::array<ScalarT, 3> voltageDerivative(IdxT terminal) const
      {
        const IdxT bus = terminal_bus_ids_[terminal];
        return {yp_[layout_.busVariable(bus, 0)],
                yp_[layout_.busVariable(bus, 1)],
                yp_[layout_.busVariable(bus, 2)]};
      }

      ScalarT input(IdxT input) const
      {
        const IdxT global = input_variable_ids_[input];
        if (global == INVALID_INDEX<IdxT>)
        {
          throw std::logic_error("EMT input is not connected");
        }
        return y_[global];
      }

    private:
      const ScalarT*        y_{nullptr};
      const ScalarT*        yp_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_bus_ids_;
      std::span<const IdxT> input_variable_ids_;
    };
  } // namespace EMT
} // namespace GridKit
