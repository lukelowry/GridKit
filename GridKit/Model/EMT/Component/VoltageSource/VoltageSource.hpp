#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

#include <GridKit/Model/EMT/Component/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
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
  } // namespace EMT
} // namespace GridKit
