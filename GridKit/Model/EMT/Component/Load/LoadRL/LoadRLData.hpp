#pragma once

#include <array>
#include <string>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type, typename index_type>
    struct LoadRLData
    {
      using RealT = real_type;
      using IdxT  = index_type;

      LoadRLData()
      {
        L.fill(RealT{1.0});
      }

      std::string device_class{"LoadRL"};
      std::string disambiguation_string;

      std::array<RealT, 3> R{};
      std::array<RealT, 3> L{};
      RealT                Iinj{0.0};
      RealT                theta{0.0};
    };
  } // namespace EMT
} // namespace GridKit
