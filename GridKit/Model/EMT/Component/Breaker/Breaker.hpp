#pragma once

#include <array>
#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <GridKit/Model/EMT/PhaseMath.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BreakerData
    {
      GridKit::Model::Events::PhaseMask closed{GridKit::Model::Events::PhaseMask::abc()};
    };

    template <class RealT, typename IdxT>
    class Breaker
    {
    public:
      static constexpr size_t variable_count = 3;
      static constexpr size_t equation_count = 3;
      static constexpr size_t terminal_count = 2;
      static constexpr size_t input_count    = 0;
      static constexpr size_t output_count   = 0;

      static constexpr size_t from = 0;
      static constexpr size_t to   = 1;

      static constexpr bool differential(size_t)
      {
        return false;
      }

      explicit Breaker(BreakerData<RealT, IdxT> data)
        : closed_{data.closed}
      {
      }

      GridKit::Model::Events::PhaseMask closed() const
      {
        return closed_;
      }

      bool closed(size_t phase) const
      {
        return closed_.includes(phase);
      }

      void open(GridKit::Model::Events::PhaseMask phases)
      {
        closed_ = closed_.without(phases);
      }

      void close(GridKit::Model::Events::PhaseMask phases)
      {
        closed_ = closed_.with(phases);
      }

      template <class Init>
      void initialize(Init& init) const
      {
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          init.setVariable(phase, RealT{0.0});
          init.setDerivative(phase, RealT{0.0});
        }
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        using Scalar = std::remove_cvref_t<decltype(x.variable(IdxT{0}))>;

        const auto v_from = x.voltage(static_cast<IdxT>(from));
        const auto v_to   = x.voltage(static_cast<IdxT>(to));

        PhaseVector<Scalar> current{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const Scalar i      = x.variable(phase);
          const Scalar status = closed_.includes(static_cast<size_t>(phase)) ? Scalar{1.0} : Scalar{0.0};

          current[static_cast<size_t>(phase)] = i;
          eq.set(phase,
                 status * (v_to[static_cast<size_t>(phase)] - v_from[static_cast<size_t>(phase)])
                     + (Scalar{1.0} - status) * i);
        }

        eq.injectCurrent(static_cast<IdxT>(from), {-current[0], -current[1], -current[2]});
        eq.injectCurrent(static_cast<IdxT>(to), current);
      }

    private:
      GridKit::Model::Events::PhaseMask closed_{GridKit::Model::Events::PhaseMask::abc()};
    };

    enum class BreakerMonitorVariable
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <class RealT, class IdxT>
    struct ComponentDescriptor<Breaker<RealT, IdxT>>
    {
      using Component = Breaker<RealT, IdxT>;
      using Data      = BreakerData<RealT, IdxT>;
      using PhaseMask = GridKit::Model::Events::PhaseMask;

      static constexpr std::string_view                class_name = "Breaker";
      static constexpr std::array<std::string_view, 2> terminals{"from", "to"};
      static constexpr std::array<std::string_view, 0> inputs{};
      static constexpr std::array<std::string_view, 0> outputs{};

      static constexpr auto params = std::tuple{
          optionalField("closed", &Data::closed, PhaseMask::abc()),
      };

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
    };

    template <class RealT, class IdxT>
    struct ComponentMonitorTraits<Breaker<RealT, IdxT>>
      : StateMonitorTable<ComponentMonitorTraits<Breaker<RealT, IdxT>>, BreakerMonitorVariable>
    {
      static constexpr auto entries = phaseCurrentMonitors<BreakerMonitorVariable>();
    };
  } // namespace EMT
} // namespace GridKit
