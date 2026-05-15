#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <tuple>

#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BranchLumpedConstantData
    {
      PhaseMatrix<RealT> r{};      // ohm / m
      PhaseMatrix<RealT> l{};      // H / m
      PhaseMatrix<RealT> g{};      // S / m
      PhaseMatrix<RealT> c{};      // F / m
      RealT              length{}; // m
    };

    template <class RealT, typename IdxT>
    class BranchLumpedConstant
    {
    public:
      static constexpr size_t variable_count = 3;
      static constexpr size_t equation_count = 3;
      static constexpr size_t terminal_count = 2;
      static constexpr size_t input_count    = 0;
      static constexpr size_t output_count   = 0;

      static constexpr size_t from = 0;
      static constexpr size_t to   = 1;

      static constexpr bool differential(size_t local)
      {
        return local < variable_count;
      }

      explicit BranchLumpedConstant(BranchLumpedConstantData<RealT, IdxT> data)
        : data_{data},
          r_{scale(data_.r, data_.length)},
          l_{scale(data_.l, data_.length)},
          g_half_{scale(data_.g, RealT{0.5} * data_.length)},
          c_half_{scale(data_.c, RealT{0.5} * data_.length)}
      {
        if (!std::isfinite(data_.length) || data_.length <= RealT{0.0})
        {
          throw std::invalid_argument("EMT BranchLumpedConstant length must be positive");
        }
        if (!isFinite(data_.r) || !isFinite(data_.l) || !isFinite(data_.g) || !isFinite(data_.c))
        {
          throw std::invalid_argument("EMT BranchLumpedConstant data must be finite");
        }
        if (isSingular(l_))
        {
          throw std::invalid_argument("EMT BranchLumpedConstant inductance matrix must be nonsingular");
        }
      }

      const BranchLumpedConstantData<RealT, IdxT>& data() const
      {
        return data_;
      }

      template <class Init>
      void initialize(Init& init) const
      {
        const RealT omega_from = static_cast<RealT>(init.omega(static_cast<IdxT>(from)));
        const RealT omega_to   = static_cast<RealT>(init.omega(static_cast<IdxT>(to)));
        if (std::abs(omega_from - omega_to) > RealT{1.0e-12} * (RealT{1.0} + std::abs(omega_from)))
        {
          throw std::invalid_argument("EMT BranchLumpedConstant terminals must share initial frequency");
        }

        const auto v_from = init.voltagePhasor(static_cast<IdxT>(from));
        const auto v_to   = init.voltagePhasor(static_cast<IdxT>(to));
        const auto z      = initialImpedance(omega_from);

        PhaseVector<std::complex<RealT>> voltage_delta{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          voltage_delta[phase] = v_from[phase] - v_to[phase];
        }

        const auto                current = solve(z, voltage_delta);
        const RealT               sqrt2   = std::sqrt(RealT{2.0});
        const std::complex<RealT> j{RealT{0.0}, RealT{1.0}};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          init.setVariable(phase, sqrt2 * current[phase].real());
          init.setDerivative(phase, (sqrt2 * j * omega_from * current[phase]).real());
        }
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        using Scalar = decltype(x.variable(IdxT{0}));

        const auto v_from  = x.voltage(static_cast<IdxT>(from));
        const auto v_to    = x.voltage(static_cast<IdxT>(to));
        const auto vp_from = x.voltageDerivative(static_cast<IdxT>(from));
        const auto vp_to   = x.voltageDerivative(static_cast<IdxT>(to));

        PhaseVector<Scalar> current{};
        PhaseVector<Scalar> current_derivative{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          current[phase]            = x.variable(phase);
          current_derivative[phase] = x.derivative(phase);
        }

        const auto r_current  = multiply(r_, current);
        const auto l_currentp = multiply(l_, current_derivative);
        const auto g_from     = multiply(g_half_, v_from);
        const auto c_from     = multiply(c_half_, vp_from);
        const auto g_to       = multiply(g_half_, v_to);
        const auto c_to       = multiply(c_half_, vp_to);

        PhaseVector<Scalar> injection_from{};
        PhaseVector<Scalar> injection_to{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          eq.set(phase, r_current[phase] + l_currentp[phase] + v_to[phase] - v_from[phase]);
          injection_from[phase] = -g_from[phase] - c_from[phase] - current[phase];
          injection_to[phase]   = -g_to[phase] - c_to[phase] + current[phase];
        }

        eq.injectCurrent(static_cast<IdxT>(from), injection_from);
        eq.injectCurrent(static_cast<IdxT>(to), injection_to);
      }

    private:
      PhaseMatrix<std::complex<RealT>> initialImpedance(RealT omega) const
      {
        PhaseMatrix<std::complex<RealT>> z{};
        for (IdxT row = 0; row < 3; ++row)
        {
          for (IdxT col = 0; col < 3; ++col)
          {
            z[row][col] = {r_[row][col], omega * l_[row][col]};
          }
        }
        return z;
      }

      BranchLumpedConstantData<RealT, IdxT> data_;
      PhaseMatrix<RealT>                    r_;
      PhaseMatrix<RealT>                    l_;
      PhaseMatrix<RealT>                    g_half_;
      PhaseMatrix<RealT>                    c_half_;
    };

    enum class BranchLumpedConstantMonitorVariable
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <class RealT, class IdxT>
    struct ComponentDescriptor<BranchLumpedConstant<RealT, IdxT>>
    {
      using Component = BranchLumpedConstant<RealT, IdxT>;
      using Data      = BranchLumpedConstantData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "BranchLumpedConstant";
      static constexpr std::array<std::string_view, 2> terminals{"from", "to"};
      static constexpr std::array<std::string_view, 0> inputs{};
      static constexpr std::array<std::string_view, 0> outputs{};

      static constexpr auto params = std::tuple{
          field("r", &Data::r),
          field("l", &Data::l),
          optionalField("g", &Data::g, PhaseMatrix<RealT>{}),
          optionalField("c", &Data::c, PhaseMatrix<RealT>{}),
          field("length", &Data::length),
      };

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
    };

    template <class RealT, class IdxT>
    struct ComponentMonitorTraits<BranchLumpedConstant<RealT, IdxT>>
      : StateMonitorTable<ComponentMonitorTraits<BranchLumpedConstant<RealT, IdxT>>,
                          BranchLumpedConstantMonitorVariable>
    {
      static constexpr auto entries = phaseCurrentMonitors<BranchLumpedConstantMonitorVariable>();
    };

    template <class RealT, class IdxT>
    struct ComponentJacobianTraits<BranchLumpedConstant<RealT, IdxT>>
    {
      static constexpr ComponentJacobianForm              form = ComponentJacobianForm::Affine;
      static constexpr ComponentJacobianCoefficientUpdate coefficient_update =
          ComponentJacobianCoefficientUpdate::Static;
    };
  } // namespace EMT
} // namespace GridKit
