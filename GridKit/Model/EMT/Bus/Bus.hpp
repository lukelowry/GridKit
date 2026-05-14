#pragma once

#include <array>
#include <cmath>
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
      static constexpr size_t variables = 3;
      static constexpr size_t equations = 3;

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
        const auto v0 = initialVoltage<ScalarT>();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[offset + phase]  = v0[phase];
          yp[offset + phase] = ScalarT{0.0};
        }
      }

      template <class ScalarT>
      std::array<ScalarT, 3> initialVoltage() const
      {
        const ScalarT pi    = std::acos(ScalarT{-1.0});
        const ScalarT shift = ScalarT{2.0} * pi / ScalarT{3.0};
        return {static_cast<ScalarT>(data_.vm0) * std::cos(static_cast<ScalarT>(data_.va0)),
                static_cast<ScalarT>(data_.vm0) * std::cos(static_cast<ScalarT>(data_.va0) - shift),
                static_cast<ScalarT>(data_.vm0) * std::cos(static_cast<ScalarT>(data_.va0) + shift)};
      }

    private:
      BusData<RealT, IdxT> data_;
    };
  } // namespace EMT
} // namespace GridKit
