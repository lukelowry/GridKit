#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace Testing
  {
    namespace EMTMocks
    {
      template <class RealT, typename IdxT>
      struct MockOneTerminalComponent
      {
        static constexpr size_t variable_count = 3;
        static constexpr size_t equation_count = 3;
        static constexpr size_t terminal_count = 1;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;

        static constexpr bool differential(size_t local)
        {
          return local < variable_count;
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto v = x.voltage(0);
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            r.set(phase, x.variable(phase) + v[phase]);
          }
          r.injectCurrent(0, {x.variable(0), x.variable(1), x.variable(2)});
        }
      };

      template <class RealT, typename IdxT>
      struct MockBranchLumpedConstant
      {
        static constexpr size_t variable_count = 3;
        static constexpr size_t equation_count = 3;
        static constexpr size_t terminal_count = 2;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;
        static constexpr size_t from           = 0;
        static constexpr size_t to             = 1;

        static constexpr bool differential(size_t local)
        {
          return local < variable_count;
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto va = x.voltage(from);
          const auto vb = x.voltage(to);
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            r.set(phase, x.variable(phase) + vb[phase] - va[phase]);
          }
          r.injectCurrent(from, {-x.variable(0), -x.variable(1), -x.variable(2)});
          r.injectCurrent(to, {x.variable(0), x.variable(1), x.variable(2)});
        }
      };

      template <class RealT, typename IdxT>
      struct MockThreeTerminalBranch
      {
        static constexpr size_t variable_count = 0;
        static constexpr size_t equation_count = 0;
        static constexpr size_t terminal_count = 3;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;

        template <class Variables, class Residual>
        void residual(const Variables&, Residual& r) const
        {
          r.injectCurrent(0, {RealT{1.0}, RealT{2.0}, RealT{3.0}});
          r.injectCurrent(1, {RealT{4.0}, RealT{5.0}, RealT{6.0}});
          r.injectCurrent(2, {RealT{7.0}, RealT{8.0}, RealT{9.0}});
        }
      };

      template <class RealT, typename IdxT>
      struct MockPortSource
      {
        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 1;

        RealT value{10.0};

        static constexpr bool differential(size_t local)
        {
          return local < variable_count;
        }

        static constexpr GridKit::EMT::SignalOutputSpec output(size_t index)
        {
          (void) index;
          return {0};
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          r.set(0, x.variable(0) - value);
        }
      };

      template <class RealT, typename IdxT>
      struct MockPortSink
      {
        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 1;
        static constexpr size_t output_count   = 0;

        static constexpr bool differential(size_t local)
        {
          return local < variable_count;
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          r.set(0, x.variable(0) - x.input(0));
        }
      };

      template <class RealT, typename IdxT>
      struct MockMissingDifferential
      {
        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          r.set(0, x.variable(0));
        }
      };

      template <class RealT, typename IdxT>
      struct MockZeroDerivativeNonlinear
      {
        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;

        static constexpr bool differential(size_t local)
        {
          return local < variable_count;
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto shifted = x.variable(0) - RealT{0.15625};
          r.set(0, shifted * shifted);
        }
      };

      template <class RealT, typename IdxT>
      struct MockDynamicComponent
      {
        static constexpr size_t variable_count = GridKit::EMT::dynamic_component_count;
        static constexpr size_t equation_count = GridKit::EMT::dynamic_component_count;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 0;

        size_t count{2};

        size_t variableCount() const
        {
          return count;
        }

        size_t equationCount() const
        {
          return count;
        }

        bool differential(size_t local) const
        {
          return local < count;
        }

        template <class Init>
        void initialize(Init& init) const
        {
          auto y  = init.y(IdxT{0}, static_cast<IdxT>(count));
          auto yp = init.yp(IdxT{0}, static_cast<IdxT>(count));
          for (size_t i = 0; i < count; ++i)
          {
            y[i]  = static_cast<RealT>(i + 1);
            yp[i] = static_cast<RealT>(100 + i);
          }
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto y  = x.y(IdxT{0}, static_cast<IdxT>(count));
          const auto yp = x.yp(IdxT{0}, static_cast<IdxT>(count));
          auto       f  = r.f(IdxT{0}, static_cast<IdxT>(count));
          for (size_t i = 0; i < count; ++i)
          {
            f[i] = yp[i] + RealT{2.0} * y[i];
          }
        }
      };
    } // namespace EMTMocks
  } // namespace Testing
} // namespace GridKit
