#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/System/Network.hpp>

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

        static constexpr GridKit::EMT::OutputSpec output(size_t index)
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
    } // namespace EMTMocks
  } // namespace Testing
} // namespace GridKit
