#pragma once

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BusData
    {
      RealT vm0{0.0};
      RealT va0{0.0};
      RealT freq{60.0};
    };
  } // namespace EMT
} // namespace GridKit
