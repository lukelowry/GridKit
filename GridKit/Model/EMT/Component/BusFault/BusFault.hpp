#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

#include <GridKit/Model/EMT/Component/BusFault/BusFaultData.hpp>
#include <GridKit/Model/EMT/PhaseMath.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class BusFault
    {
    public:
      static constexpr size_t variable_count = 0;
      static constexpr size_t equation_count = 0;
      static constexpr size_t terminal_count = 1;
      static constexpr size_t input_count    = 0;
      static constexpr size_t output_count   = 0;

      explicit BusFault(BusFaultData<RealT, IdxT> data)
        : active_{data.active},
          r_{data.r},
          minimum_resistance_{data.minimum_resistance}
      {
        validateResistance(minimum_resistance_, "EMT BusFault minimum resistance must be positive and finite");
        validateResistance(r_, "EMT BusFault resistance must be positive and finite");
        if (r_ < minimum_resistance_)
        {
          r_ = minimum_resistance_;
        }
      }

      GridKit::Model::Events::PhaseMask active() const
      {
        return active_;
      }

      bool active(size_t phase) const
      {
        return active_.includes(phase);
      }

      RealT resistance() const
      {
        return r_;
      }

      void fault(const GridKit::Model::Events::Fault& action)
      {
        if (action.x != 0.0)
        {
          throw std::invalid_argument("EMT BusFault currently supports resistive faults only");
        }

        const RealT r = static_cast<RealT>(action.r);
        validateResistance(r, "EMT BusFault event resistance must be positive and finite");
        r_      = r < minimum_resistance_ ? minimum_resistance_ : r;
        active_ = active_.with(action.phases);
      }

      void clear(GridKit::Model::Events::PhaseMask phases)
      {
        active_ = active_.without(phases);
      }

      template <class ScalarT>
      ScalarT current(ScalarT voltage, size_t phase) const
      {
        if (!active_.includes(phase))
        {
          return ScalarT{0.0};
        }
        return -voltage / static_cast<ScalarT>(r_);
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        using Scalar = std::remove_cvref_t<decltype(x.voltage(IdxT{0})[0])>;

        const auto          v = x.voltage(IdxT{0});
        PhaseVector<Scalar> injection{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          injection[static_cast<size_t>(phase)] =
              current(static_cast<Scalar>(v[static_cast<size_t>(phase)]),
                      static_cast<size_t>(phase));
        }
        eq.injectCurrent(IdxT{0}, injection);
      }

    private:
      static void validateResistance(RealT value, const char* message)
      {
        if (!std::isfinite(value) || value <= RealT{0.0})
        {
          throw std::invalid_argument(message);
        }
      }

      GridKit::Model::Events::PhaseMask active_{GridKit::Model::Events::PhaseMask::none()};
      RealT                             r_{RealT{1.0}};
      RealT                             minimum_resistance_{RealT{1.0e-6}};
    };
  } // namespace EMT
} // namespace GridKit
