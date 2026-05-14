#pragma once

#include <array>
#include <complex>
#include <initializer_list>
#include <span>
#include <stdexcept>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class InitialStateView
    {
    public:
      using BusT = Bus<ScalarT, IdxT>;

      InitialStateView(ScalarT*              y,
                       ScalarT*              yp,
                       const Layout<IdxT>&   layout,
                       ComponentLayout<IdxT> component,
                       std::span<const IdxT> terminal_buses,
                       std::span<const BusT> buses)
        : y_{y},
          yp_{yp},
          layout_{layout},
          component_{component},
          terminal_buses_{terminal_buses},
          buses_{buses}
      {
      }

      void setVariable(IdxT local, ScalarT value)
      {
        y_[component_.variable_offset + local] = value;
      }

      void setDerivative(IdxT local, ScalarT value)
      {
        yp_[component_.variable_offset + local] = value;
      }

      ScalarT variable(IdxT local) const
      {
        return y_[component_.variable_offset + local];
      }

      ScalarT derivative(IdxT local) const
      {
        return yp_[component_.variable_offset + local];
      }

      std::array<std::complex<ScalarT>, 3> voltagePhasor(IdxT terminal) const
      {
        return buses_[terminal_buses_[terminal]].template initialVoltagePhasor<ScalarT>();
      }

      ScalarT omega(IdxT terminal) const
      {
        return buses_[terminal_buses_[terminal]].template omega<ScalarT>();
      }

      std::array<ScalarT, 3> voltage(IdxT terminal) const
      {
        const IdxT bus = terminal_buses_[terminal];
        return {y_[layout_.busVariable(bus, 0)],
                y_[layout_.busVariable(bus, 1)],
                y_[layout_.busVariable(bus, 2)]};
      }

      std::array<ScalarT, 3> voltageDerivative(IdxT terminal) const
      {
        const IdxT bus = terminal_buses_[terminal];
        return {yp_[layout_.busVariable(bus, 0)],
                yp_[layout_.busVariable(bus, 1)],
                yp_[layout_.busVariable(bus, 2)]};
      }

    private:
      ScalarT*              y_{nullptr};
      ScalarT*              yp_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_buses_;
      std::span<const BusT> buses_;
    };

    template <class ScalarT, typename IdxT>
    class StateView
    {
    public:
      using scalar_type = ScalarT;
      using index_type  = IdxT;

      StateView(const ScalarT*        y,
                const ScalarT*        yp,
                const Layout<IdxT>&   layout,
                ComponentLayout<IdxT> component,
                std::span<const IdxT> terminal_buses,
                std::span<const IdxT> input_variables,
                ScalarT               time)
        : y_{y},
          yp_{yp},
          layout_{layout},
          component_{component},
          terminal_buses_{terminal_buses},
          input_variables_{input_variables},
          time_{time}
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
        const IdxT bus = terminal_buses_[terminal];
        return {y_[layout_.busVariable(bus, 0)],
                y_[layout_.busVariable(bus, 1)],
                y_[layout_.busVariable(bus, 2)]};
      }

      std::array<ScalarT, 3> voltageDerivative(IdxT terminal) const
      {
        const IdxT bus = terminal_buses_[terminal];
        return {yp_[layout_.busVariable(bus, 0)],
                yp_[layout_.busVariable(bus, 1)],
                yp_[layout_.busVariable(bus, 2)]};
      }

      ScalarT input(IdxT input) const
      {
        const IdxT global = input_variables_[input];
        if (global == INVALID_INDEX<IdxT>)
        {
          throw std::logic_error("EMT input is not connected");
        }
        return y_[global];
      }

      ScalarT time() const
      {
        return time_;
      }

    private:
      const ScalarT*        y_{nullptr};
      const ScalarT*        yp_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_buses_;
      std::span<const IdxT> input_variables_;
      ScalarT               time_{0.0};
    };

    template <class ScalarT, typename IdxT>
    class EquationView
    {
    public:
      EquationView(ScalarT*              f,
                   const Layout<IdxT>&   layout,
                   ComponentLayout<IdxT> component,
                   std::span<const IdxT> terminal_buses)
        : f_{f},
          layout_{layout},
          component_{component},
          terminal_buses_{terminal_buses}
      {
      }

      template <class ValueT>
      void set(IdxT local, ValueT value)
      {
        f_[component_.equation_offset + local] = static_cast<ScalarT>(value);
      }

      template <class ValueT>
      void add(IdxT local, ValueT value)
      {
        f_[component_.equation_offset + local] += static_cast<ScalarT>(value);
      }

      template <class ValueT, size_t N>
      void set(IdxT first, const std::array<ValueT, N>& values)
      {
        for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
        {
          set(first + i, values[i]);
        }
      }

      template <class ValueT>
      void injectCurrent(IdxT terminal, const std::array<ValueT, 3>& current)
      {
        const IdxT bus = terminal_buses_[terminal];
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          f_[layout_.busEquation(bus, phase)] += static_cast<ScalarT>(current[phase]);
        }
      }

      template <class ValueT>
      void injectCurrent(IdxT terminal, std::initializer_list<ValueT> current)
      {
        if (current.size() != 3)
        {
          throw std::invalid_argument("EMT current injection must have three phases");
        }

        const IdxT bus   = terminal_buses_[terminal];
        auto       value = current.begin();
        for (IdxT phase = 0; phase < 3; ++phase, ++value)
        {
          f_[layout_.busEquation(bus, phase)] += static_cast<ScalarT>(*value);
        }
      }

    private:
      ScalarT*              f_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> terminal_buses_;
    };

    namespace Detail
    {
      template <class ScalarT, typename IdxT>
      class LocalStateView
      {
      public:
        LocalStateView(const ScalarT* y,
                       const ScalarT* yp,
                       const ScalarT* terminal_v,
                       const ScalarT* terminal_vp,
                       const ScalarT* inputs,
                       ScalarT        time)
          : y_{y},
            yp_{yp},
            terminal_v_{terminal_v},
            terminal_vp_{terminal_vp},
            inputs_{inputs},
            time_{time}
        {
        }

        ScalarT variable(IdxT local) const
        {
          return y_[local];
        }

        ScalarT derivative(IdxT local) const
        {
          return yp_[local];
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
          const IdxT first = 3 * terminal;
          return {terminal_v_[first + 0], terminal_v_[first + 1], terminal_v_[first + 2]};
        }

        std::array<ScalarT, 3> voltageDerivative(IdxT terminal) const
        {
          const IdxT first = 3 * terminal;
          return {terminal_vp_[first + 0], terminal_vp_[first + 1], terminal_vp_[first + 2]};
        }

        ScalarT input(IdxT input) const
        {
          return inputs_[input];
        }

        ScalarT time() const
        {
          return time_;
        }

      private:
        const ScalarT* y_{nullptr};
        const ScalarT* yp_{nullptr};
        const ScalarT* terminal_v_{nullptr};
        const ScalarT* terminal_vp_{nullptr};
        const ScalarT* inputs_{nullptr};
        ScalarT        time_{0.0};
      };

      template <class ScalarT, typename IdxT>
      class LocalEquationView
      {
      public:
        LocalEquationView(ScalarT* residual, IdxT equation_count)
          : residual_{residual},
            equation_count_{equation_count}
        {
        }

        template <class ValueT>
        void set(IdxT local, ValueT value)
        {
          residual_[local] = static_cast<ScalarT>(value);
        }

        template <class ValueT>
        void add(IdxT local, ValueT value)
        {
          residual_[local] += static_cast<ScalarT>(value);
        }

        template <class ValueT, size_t N>
        void set(IdxT first, const std::array<ValueT, N>& values)
        {
          for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
          {
            set(first + i, values[i]);
          }
        }

        template <class ValueT>
        void injectCurrent(IdxT terminal, const std::array<ValueT, 3>& current)
        {
          const IdxT first = equation_count_ + 3 * terminal;
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            residual_[first + phase] += static_cast<ScalarT>(current[phase]);
          }
        }

        template <class ValueT>
        void injectCurrent(IdxT terminal, std::initializer_list<ValueT> current)
        {
          if (current.size() != 3)
          {
            throw std::invalid_argument("EMT current injection must have three phases");
          }

          const IdxT first = equation_count_ + 3 * terminal;
          auto       value = current.begin();
          for (IdxT phase = 0; phase < 3; ++phase, ++value)
          {
            residual_[first + phase] += static_cast<ScalarT>(*value);
          }
        }

      private:
        ScalarT* residual_{nullptr};
        IdxT     equation_count_{0};
      };
    } // namespace Detail
  } // namespace EMT
} // namespace GridKit
