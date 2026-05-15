#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>

#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BusData
    {
      RealT vm{0.0}; ///< Initial RMS phase voltage magnitude
      RealT va{0.0}; ///< Initial phase-a voltage angle [rad]
    };

    template <class RealT, typename IdxT>
    class Bus
    {
    public:
      struct FaultState
      {
        GridKit::Model::Events::PhaseMask active{GridKit::Model::Events::PhaseMask::none()};
        RealT                             r{RealT{1.0}};
        RealT                             minimum_resistance{RealT{1.0e-6}};
        bool                              structural{false};
      };

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
        const ScalarT vm    = static_cast<ScalarT>(data_.vm);
        const ScalarT va    = static_cast<ScalarT>(data_.va);
        return {std::polar(vm, va), std::polar(vm, va - shift), std::polar(vm, va + shift)};
      }

      template <class ScalarT>
      ScalarT omega() const
      {
        const ScalarT pi = std::acos(ScalarT{-1.0});
        return ScalarT{2.0} * pi * ScalarT{60.0};
      }

      void fault(const GridKit::Model::Events::Fault& action)
      {
        if (action.x != 0.0 || action.percent != 0.0)
        {
          throw std::invalid_argument("EMT bus faults currently support resistance only");
        }

        const RealT r = static_cast<RealT>(action.r);
        validateFaultResistance(r, "EMT bus fault resistance must be positive and finite");

        fault_.r      = std::max(r, fault_.minimum_resistance);
        fault_.active = fault_.active.with(action.phases);
      }

      void clear(GridKit::Model::Events::PhaseMask phases)
      {
        fault_.active = fault_.active.without(phases);
      }

      bool faultActive(size_t phase) const
      {
        return fault_.active.includes(phase);
      }

      bool hasFaultState() const
      {
        return fault_.structural || !fault_.active.empty();
      }

      RealT faultResistance() const
      {
        return fault_.r;
      }

      void prepareFaultStructure()
      {
        fault_.structural = true;
      }

      template <class ScalarT>
      ScalarT faultCurrent(ScalarT voltage, size_t phase) const
      {
        if (!faultActive(phase))
        {
          return ScalarT{0.0};
        }
        return -voltage / static_cast<ScalarT>(fault_.r);
      }

      template <class ScalarT>
      ScalarT residualCurrent(ScalarT voltage, size_t phase) const
      {
        return faultCurrent(voltage, phase);
      }

      RealT residualJacobian(size_t phase) const
      {
        if (!faultActive(phase))
        {
          return RealT{0.0};
        }
        return -RealT{1.0} / fault_.r;
      }

    private:
      static void validateFaultResistance(RealT value, const char* message)
      {
        if (!std::isfinite(value) || value <= RealT{0.0})
        {
          throw std::invalid_argument(message);
        }
      }

      BusData<RealT, IdxT> data_;
      FaultState           fault_{};
    };
  } // namespace EMT
} // namespace GridKit
