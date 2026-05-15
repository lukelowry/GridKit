#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <tuple>

#include <GridKit/Model/EMT/PhaseMath.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct LoadRLData
    {
      PhaseVector<RealT> r{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> l{RealT{0.0}, RealT{0.0}, RealT{0.0}};
    };

    template <class RealT, typename IdxT>
    class LoadRL
    {
    public:
      static constexpr size_t variable_count = 3;
      static constexpr size_t equation_count = 3;
      static constexpr size_t terminal_count = 1;
      static constexpr size_t input_count    = 0;
      static constexpr size_t output_count   = 0;

      static constexpr bool differential(size_t local)
      {
        return local < variable_count;
      }

      explicit LoadRL(LoadRLData<RealT, IdxT> data)
        : data_{data}
      {
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          if (data_.r[phase] < RealT{0.0})
          {
            throw std::invalid_argument("EMT LoadRL resistance must be nonnegative");
          }
          if (data_.l[phase] <= RealT{0.0})
          {
            throw std::invalid_argument("EMT LoadRL inductance must be positive");
          }
        }
      }

      const LoadRLData<RealT, IdxT>& data() const
      {
        return data_;
      }

      template <class Init>
      void initialize(Init& init) const
      {
        const auto                voltage = init.voltagePhasor(0);
        const RealT               omega   = static_cast<RealT>(init.omega(0));
        const RealT               sqrt2   = std::sqrt(RealT{2.0});
        const std::complex<RealT> j{RealT{0.0}, RealT{1.0}};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT               r_phase = data_.r[phase];
          const RealT               l_phase = data_.l[phase];
          const std::complex<RealT> impedance{r_phase, omega * l_phase};
          if (std::norm(impedance) == RealT{0.0})
          {
            throw std::invalid_argument("EMT LoadRL initial impedance must be nonzero");
          }

          const auto current = -voltage[phase] / impedance;
          init.setVariable(phase, sqrt2 * current.real());
          init.setDerivative(phase, (sqrt2 * j * omega * current).real());
        }
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        using Scalar            = decltype(x.variable(IdxT{0}));
        const auto            v = x.voltage(0);
        std::array<Scalar, 3> current{};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const Scalar i       = x.variable(phase);
          const Scalar ip      = x.derivative(phase);
          const Scalar r_phase = static_cast<Scalar>(data_.r[phase]);
          const Scalar l_phase = static_cast<Scalar>(data_.l[phase]);

          current[phase] = i;
          eq.set(phase, r_phase * i + l_phase * ip + v[phase]);
        }
        eq.injectCurrent(0, current);
      }

    private:
      LoadRLData<RealT, IdxT> data_;
    };

    enum class LoadRLMonitorVariable
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <class RealT, class IdxT>
    struct ComponentDescriptor<LoadRL<RealT, IdxT>>
    {
      using Component = LoadRL<RealT, IdxT>;
      using Data      = LoadRLData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "LoadRL";
      static constexpr std::array<std::string_view, 1> terminals{"ac"};
      static constexpr std::array<std::string_view, 0> inputs{};
      static constexpr std::array<std::string_view, 0> outputs{};

      static constexpr auto params = std::tuple{
          field("r", &Data::r),
          field("l", &Data::l),
      };

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
    };

    template <class RealT, class IdxT>
    struct ComponentMonitorTraits<LoadRL<RealT, IdxT>>
      : StateMonitorTable<ComponentMonitorTraits<LoadRL<RealT, IdxT>>, LoadRLMonitorVariable>
    {
      static constexpr auto entries = phaseCurrentMonitors<LoadRLMonitorVariable>();
    };

    template <class RealT, class IdxT>
    struct ComponentJacobianTraits<LoadRL<RealT, IdxT>>
    {
      static constexpr ComponentJacobianForm              form = ComponentJacobianForm::Affine;
      static constexpr ComponentJacobianCoefficientUpdate coefficient_update =
          ComponentJacobianCoefficientUpdate::Static;
    };
  } // namespace EMT
} // namespace GridKit
