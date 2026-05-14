#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/System/Port.hpp>

namespace GridKit
{
  namespace Testing
  {
    namespace EMTMocks
    {
      template <class RealT, typename IdxT>
      struct MockOneTerminalComponent
      {
        static constexpr size_t variables       = 3;
        static constexpr size_t equations       = 3;
        static constexpr size_t terminals       = 1;
        static constexpr size_t inputs          = 0;
        static constexpr size_t outputs         = 0;
        static constexpr bool   direct_jacobian = true;

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto v = x.voltage(0);
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            r.equation(phase, x.variable(phase) + v[phase]);
          }
          r.inject(0, {x.variable(0), x.variable(1), x.variable(2)});
        }

        template <class Pattern>
        void pattern(Pattern& p) const
        {
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            p.addEquationVariable(phase, phase);
            p.addEquationTerminalVoltage(phase, 0, phase);
            p.addInjectionVariable(0, phase, phase);
          }
        }

        template <class Variables, class Jacobian>
        void jacobian(const Variables&, Jacobian& j) const
        {
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            j.addEquationVariable(phase, phase, RealT{1.0});
            j.addEquationTerminalVoltage(phase, 0, phase, RealT{1.0});
            j.addInjectionVariable(0, phase, phase, RealT{1.0});
          }
        }
      };

      template <class RealT, typename IdxT>
      struct MockBranchLumpedConstant
      {
        static constexpr size_t variables       = 3;
        static constexpr size_t equations       = 3;
        static constexpr size_t terminals       = 2;
        static constexpr size_t inputs          = 0;
        static constexpr size_t outputs         = 0;
        static constexpr bool   direct_jacobian = true;
        static constexpr size_t from            = 0;
        static constexpr size_t to              = 1;

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          const auto va = x.voltage(from);
          const auto vb = x.voltage(to);
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            r.equation(phase, x.variable(phase) + vb[phase] - va[phase]);
          }
          r.inject(from, {-x.variable(0), -x.variable(1), -x.variable(2)});
          r.inject(to, {x.variable(0), x.variable(1), x.variable(2)});
        }

        template <class Pattern>
        void pattern(Pattern& p) const
        {
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            p.addEquationVariable(phase, phase);
            p.addEquationTerminalVoltage(phase, from, phase);
            p.addEquationTerminalVoltage(phase, to, phase);
            p.addInjectionVariable(from, phase, phase);
            p.addInjectionVariable(to, phase, phase);
          }
        }

        template <class Variables, class Jacobian>
        void jacobian(const Variables&, Jacobian& j) const
        {
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            j.addEquationVariable(phase, phase, RealT{1.0});
            j.addEquationTerminalVoltage(phase, from, phase, RealT{-1.0});
            j.addEquationTerminalVoltage(phase, to, phase, RealT{1.0});
            j.addInjectionVariable(from, phase, phase, RealT{-1.0});
            j.addInjectionVariable(to, phase, phase, RealT{1.0});
          }
        }
      };

      template <class RealT, typename IdxT>
      struct MockThreeTerminalBranch
      {
        static constexpr size_t variables       = 0;
        static constexpr size_t equations       = 0;
        static constexpr size_t terminals       = 3;
        static constexpr size_t inputs          = 0;
        static constexpr size_t outputs         = 0;
        static constexpr bool   direct_jacobian = true;

        template <class Variables, class Residual>
        void residual(const Variables&, Residual& r) const
        {
          r.inject(0, {RealT{1.0}, RealT{2.0}, RealT{3.0}});
          r.inject(1, {RealT{4.0}, RealT{5.0}, RealT{6.0}});
          r.inject(2, {RealT{7.0}, RealT{8.0}, RealT{9.0}});
        }

        template <class Pattern>
        void pattern(Pattern&) const
        {
        }

        template <class Variables, class Jacobian>
        void jacobian(const Variables&, Jacobian&) const
        {
        }
      };

      template <class RealT, typename IdxT>
      struct MockPortSource
      {
        static constexpr size_t variables       = 1;
        static constexpr size_t equations       = 1;
        static constexpr size_t terminals       = 0;
        static constexpr size_t inputs          = 0;
        static constexpr size_t outputs         = 1;
        static constexpr bool   direct_jacobian = true;

        RealT value{10.0};

        static constexpr GridKit::EMT::OutputSpec output(size_t index)
        {
          (void) index;
          return {0};
        }

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          r.equation(0, x.variable(0) - value);
        }

        template <class Pattern>
        void pattern(Pattern& p) const
        {
          p.addEquationVariable(0, 0);
        }

        template <class Variables, class Jacobian>
        void jacobian(const Variables&, Jacobian& j) const
        {
          j.addEquationVariable(0, 0, RealT{1.0});
        }
      };

      template <class RealT, typename IdxT>
      struct MockPortSink
      {
        static constexpr size_t variables       = 1;
        static constexpr size_t equations       = 1;
        static constexpr size_t terminals       = 0;
        static constexpr size_t inputs          = 1;
        static constexpr size_t outputs         = 0;
        static constexpr bool   direct_jacobian = true;

        template <class Variables, class Residual>
        void residual(const Variables& x, Residual& r) const
        {
          r.equation(0, x.variable(0) - x.input(0));
        }

        template <class Pattern>
        void pattern(Pattern& p) const
        {
          p.addEquationVariable(0, 0);
          p.addEquationInput(0, 0);
        }

        template <class Variables, class Jacobian>
        void jacobian(const Variables&, Jacobian& j) const
        {
          j.addEquationVariable(0, 0, RealT{1.0});
          j.addEquationInput(0, 0, RealT{-1.0});
        }
      };
    } // namespace EMTMocks
  } // namespace Testing
} // namespace GridKit
