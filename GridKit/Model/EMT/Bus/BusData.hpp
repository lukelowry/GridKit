#pragma once

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BusData
    {
      RealT vm0{0.0};   ///< Initial RMS phase voltage magnitude
      RealT va0{0.0};   ///< Initial phase-a voltage angle [rad]
      RealT freq{60.0}; ///< Initial bus frequency [Hz]
    };
  } // namespace EMT
} // namespace GridKit
