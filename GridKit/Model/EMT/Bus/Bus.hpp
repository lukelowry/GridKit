#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>

#include <GridKit/Model/EMT/Bus/BusData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class Bus
    {
    public:
      static constexpr size_t variable_count = 3;
      static constexpr size_t equation_count = 3;

      Bus() = default;

      explicit Bus(BusData<RealT, IdxT> data)
        : data_{data}
      {
      }

      const BusData<RealT, IdxT>& data() const
      {
        return data_;
      }

      template <class ScalarT>
      void initialize(ScalarT* y, ScalarT* yp, IdxT offset) const
      {
        const auto v0  = initialVoltage<ScalarT>();
        const auto vp0 = initialVoltageDerivative<ScalarT>();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[offset + phase]  = v0[phase];
          yp[offset + phase] = vp0[phase];
        }
      }

      template <class ScalarT>
      std::array<ScalarT, 3> initialVoltage() const
      {
        const ScalarT sqrt2 = std::sqrt(ScalarT{2.0});
        const auto    v     = initialVoltagePhasor<ScalarT>();
        return {sqrt2 * v[0].real(), sqrt2 * v[1].real(), sqrt2 * v[2].real()};
      }

      template <class ScalarT>
      std::array<ScalarT, 3> initialVoltageDerivative() const
      {
        const ScalarT sqrt2 = std::sqrt(ScalarT{2.0});
        const ScalarT w     = omega<ScalarT>();
        const auto    v     = initialVoltagePhasor<ScalarT>();
        return {-sqrt2 * w * v[0].imag(), -sqrt2 * w * v[1].imag(), -sqrt2 * w * v[2].imag()};
      }

      template <class ScalarT>
      std::array<std::complex<ScalarT>, 3> initialVoltagePhasor() const
      {
        const ScalarT pi    = std::acos(ScalarT{-1.0});
        const ScalarT shift = ScalarT{2.0} * pi / ScalarT{3.0};
        const ScalarT vm    = static_cast<ScalarT>(data_.vm0);
        const ScalarT va    = static_cast<ScalarT>(data_.va0);
        return {std::polar(vm, va), std::polar(vm, va - shift), std::polar(vm, va + shift)};
      }

      template <class ScalarT>
      ScalarT omega() const
      {
        const ScalarT pi = std::acos(ScalarT{-1.0});
        return ScalarT{2.0} * pi * static_cast<ScalarT>(data_.freq);
      }

    private:
      BusData<RealT, IdxT> data_;
    };
  } // namespace EMT
} // namespace GridKit
