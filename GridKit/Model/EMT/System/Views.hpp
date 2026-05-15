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
                       std::span<const IdxT> port_buses,
                       std::span<const BusT> buses)
        : y_{y},
          yp_{yp},
          layout_{layout},
          component_{component},
          port_buses_{port_buses},
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

      std::span<ScalarT> y(IdxT first, IdxT count)
      {
        return {y_ + component_.variable_offset + first, static_cast<size_t>(count)};
      }

      std::span<ScalarT> yp(IdxT first, IdxT count)
      {
        return {yp_ + component_.variable_offset + first, static_cast<size_t>(count)};
      }

      std::array<std::complex<ScalarT>, 3> voltagePhasor(IdxT port) const
      {
        return buses_[port_buses_[port]].template initialVoltagePhasor<ScalarT>();
      }

      ScalarT omega(IdxT port) const
      {
        return buses_[port_buses_[port]].template omega<ScalarT>();
      }

      std::array<ScalarT, 3> voltage(IdxT port) const
      {
        const IdxT bus = port_buses_[port];
        return {y_[layout_.busVariable(bus, 0)],
                y_[layout_.busVariable(bus, 1)],
                y_[layout_.busVariable(bus, 2)]};
      }

      std::array<ScalarT, 3> voltageDerivative(IdxT port) const
      {
        const IdxT bus = port_buses_[port];
        return {yp_[layout_.busVariable(bus, 0)],
                yp_[layout_.busVariable(bus, 1)],
                yp_[layout_.busVariable(bus, 2)]};
      }

    private:
      ScalarT*              y_{nullptr};
      ScalarT*              yp_{nullptr};
      const Layout<IdxT>&   layout_;
      ComponentLayout<IdxT> component_;
      std::span<const IdxT> port_buses_;
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
                std::span<const IdxT> port_buses,
                std::span<const IdxT> input_port_variables,
                ScalarT               time)
        : y_{y},
          yp_{yp},
          layout_{layout},
          component_{component},
          port_buses_{port_buses},
          input_port_variables_{input_port_variables},
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

      std::span<const ScalarT> y(IdxT first, IdxT count) const
      {
        return {y_ + component_.variable_offset + first, static_cast<size_t>(count)};
      }

      std::span<const ScalarT> yp(IdxT first, IdxT count) const
      {
        return {yp_ + component_.variable_offset + first, static_cast<size_t>(count)};
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

      std::array<ScalarT, 3> voltage(IdxT port) const
      {
        const IdxT bus = port_buses_[port];
        return {y_[layout_.busVariable(bus, 0)],
                y_[layout_.busVariable(bus, 1)],
                y_[layout_.busVariable(bus, 2)]};
      }

      std::array<ScalarT, 3> voltageDerivative(IdxT port) const
      {
        const IdxT bus = port_buses_[port];
        return {yp_[layout_.busVariable(bus, 0)],
                yp_[layout_.busVariable(bus, 1)],
                yp_[layout_.busVariable(bus, 2)]};
      }

      ScalarT inputPort(IdxT input) const
      {
        const IdxT global = input_port_variables_[input];
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
      std::span<const IdxT> port_buses_;
      std::span<const IdxT> input_port_variables_;
      ScalarT               time_{0.0};
    };

    template <class ScalarT, typename IdxT>
    class EquationView
    {
    public:
      EquationView(ScalarT*              f,
                   const Layout<IdxT>&   layout,
                   ComponentLayout<IdxT> component,
                   std::span<const IdxT> port_buses)
        : f_{f},
          layout_{layout},
          component_{component},
          port_buses_{port_buses}
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

      std::span<ScalarT> f(IdxT first, IdxT count)
      {
        return {f_ + component_.equation_offset + first, static_cast<size_t>(count)};
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
      void injectCurrent(IdxT port, const std::array<ValueT, 3>& current)
      {
        const IdxT bus = port_buses_[port];
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          f_[layout_.busEquation(bus, phase)] += static_cast<ScalarT>(current[phase]);
        }
      }

      template <class ValueT>
      void injectCurrent(IdxT port, std::initializer_list<ValueT> current)
      {
        if (current.size() != 3)
        {
          throw std::invalid_argument("EMT current injection must have three phases");
        }

        const IdxT bus   = port_buses_[port];
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
      std::span<const IdxT> port_buses_;
    };

    namespace Detail
    {
      template <class ScalarT, typename IdxT>
      class LocalStateView
      {
      public:
        LocalStateView(const ScalarT* y,
                       const ScalarT* yp,
                       const ScalarT* port_v,
                       const ScalarT* port_vp,
                       const ScalarT* input_ports,
                       ScalarT        time)
          : y_{y},
            yp_{yp},
            port_v_{port_v},
            port_vp_{port_vp},
            input_ports_{input_ports},
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

        std::span<const ScalarT> y(IdxT first, IdxT count) const
        {
          return {y_ + first, static_cast<size_t>(count)};
        }

        std::span<const ScalarT> yp(IdxT first, IdxT count) const
        {
          return {yp_ + first, static_cast<size_t>(count)};
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

        std::array<ScalarT, 3> voltage(IdxT port) const
        {
          const IdxT first = 3 * port;
          return {port_v_[first + 0], port_v_[first + 1], port_v_[first + 2]};
        }

        std::array<ScalarT, 3> voltageDerivative(IdxT port) const
        {
          const IdxT first = 3 * port;
          return {port_vp_[first + 0], port_vp_[first + 1], port_vp_[first + 2]};
        }

        ScalarT inputPort(IdxT input) const
        {
          return input_ports_[input];
        }

        ScalarT time() const
        {
          return time_;
        }

      private:
        const ScalarT* y_{nullptr};
        const ScalarT* yp_{nullptr};
        const ScalarT* port_v_{nullptr};
        const ScalarT* port_vp_{nullptr};
        const ScalarT* input_ports_{nullptr};
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

        std::span<ScalarT> f(IdxT first, IdxT count)
        {
          return {residual_ + first, static_cast<size_t>(count)};
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
        void injectCurrent(IdxT port, const std::array<ValueT, 3>& current)
        {
          const IdxT first = equation_count_ + 3 * port;
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            residual_[first + phase] += static_cast<ScalarT>(current[phase]);
          }
        }

        template <class ValueT>
        void injectCurrent(IdxT port, std::initializer_list<ValueT> current)
        {
          if (current.size() != 3)
          {
            throw std::invalid_argument("EMT current injection must have three phases");
          }

          const IdxT first = equation_count_ + 3 * port;
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
