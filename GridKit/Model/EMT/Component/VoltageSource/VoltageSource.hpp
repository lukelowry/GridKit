#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <GridKit/Model/EMT/PhaseMath.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct VoltageSourceData
    {
      PhaseVector<RealT> e{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> phi{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> r{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      RealT              omega0{0.0};
    };

    template <class RealT, typename IdxT>
    class VoltageSource
    {
    public:
      static constexpr size_t variable_count = 0;
      static constexpr size_t equation_count = 0;
      static constexpr size_t terminal_count = 1;
      static constexpr size_t input_count    = 0;
      static constexpr size_t output_count   = 0;

      explicit VoltageSource(VoltageSourceData<RealT, IdxT> data)
        : data_{data}
      {
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          if (data_.e[phase] < RealT{0.0})
          {
            throw std::invalid_argument("EMT VoltageSource RMS magnitude must be nonnegative");
          }
          if (data_.r[phase] <= RealT{0.0})
          {
            throw std::invalid_argument("EMT VoltageSource terminal resistance must be positive");
          }
        }
      }

      const VoltageSourceData<RealT, IdxT>& data() const
      {
        return data_;
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        using Scalar = std::remove_cvref_t<decltype(x.voltage(0)[0])>;

        const auto            v      = x.voltage(0);
        const auto            sqrt2  = std::sqrt(RealT{2.0});
        const RealT           t      = static_cast<RealT>(x.time());
        const RealT           omega0 = data_.omega0;
        std::array<Scalar, 3> current{};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT  e_rms   = data_.e[phase];
          const RealT  phi     = data_.phi[phase];
          const Scalar r_phase = static_cast<Scalar>(data_.r[phase]);
          const Scalar e       = static_cast<Scalar>(sqrt2 * e_rms * std::cos(omega0 * t + phi));

          current[phase] = (e - v[phase]) / r_phase;
        }
        eq.injectCurrent(0, current);
      }

    private:
      VoltageSourceData<RealT, IdxT> data_;
    };

    template <class RealT>
    RealT hzToRadPerSec(RealT hz)
    {
      return RealT{2.0} * std::acos(RealT{-1.0}) * hz;
    }

    enum class VoltageSourceMonitorVariable
    {
      ia,
      ib,
      ic
    };

    template <class RealT, class IdxT>
    struct ComponentDescriptor<VoltageSource<RealT, IdxT>>
    {
      using Component = VoltageSource<RealT, IdxT>;
      using Data      = VoltageSourceData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "VoltageSource";
      static constexpr std::array<std::string_view, 1> terminals{"ac"};
      static constexpr std::array<std::string_view, 0> inputs{};
      static constexpr std::array<std::string_view, 0> outputs{};

      static constexpr auto params = std::tuple{
          field("e", &Data::e),
          field("phi", &Data::phi),
          field("r", &Data::r),
          field("frequency", &Data::omega0, &hzToRadPerSec<RealT>),
      };

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
    };

    template <class RealT, class IdxT>
    struct ComponentMonitorTraits<VoltageSource<RealT, IdxT>>
    {
      using Variable = VoltageSourceMonitorVariable;

      template <class Context>
      static void bind(Context& ctx, const VoltageSource<RealT, IdxT>& source, Variable variable)
      {
        IdxT        phase = 0;
        const char* label = nullptr;
        switch (variable)
        {
        case Variable::ia:
          phase = 0;
          label = "ia";
          break;
        case Variable::ib:
          phase = 1;
          label = "ib";
          break;
        case Variable::ic:
          phase = 2;
          label = "ic";
          break;
        default:
          throw std::invalid_argument("Invalid EMT VoltageSource monitor variable");
        }

        const auto data        = source.data();
        const auto bus_voltage = ctx.terminalVoltageIndex(0, phase);
        const auto y           = ctx.y();
        const auto time        = ctx.time();

        ctx.add(label,
                [data, y, time, bus_voltage, phase]()
                {
                  const RealT sqrt2 = std::sqrt(RealT{2.0});
                  const RealT e     = sqrt2 * data.e[phase] * std::cos(data.omega0 * (*time) + data.phi[phase]);
                  const RealT v     = static_cast<RealT>((*y)[static_cast<std::size_t>(bus_voltage)]);
                  return (e - v) / data.r[phase];
                });
      }

      static std::optional<Variable> resolve(std::string_view name)
      {
        return resolveMonitorVariable<Variable>(name);
      }
    };
  } // namespace EMT
} // namespace GridKit
