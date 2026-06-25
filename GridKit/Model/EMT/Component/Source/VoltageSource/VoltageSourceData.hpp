#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type, typename index_type, std::size_t N>
    struct VoltageSourceData
    {
      using RealT = real_type;
      using IdxT  = index_type;

      VoltageSourceData()
      {
        G.fill(RealT{1.0});
        omega = RealT{1.0};
      }

      std::string device_class{"VoltageSource"};
      std::string disambiguation_string;

      std::array<RealT, N> E{};
      std::array<RealT, N> phi{};
      RealT                omega{1.0};
      std::array<RealT, N> G{};
    };
  } // namespace EMT
} // namespace GridKit
