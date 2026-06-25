#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type, typename index_type, std::size_t N = 3>
    struct BusData
    {
      using RealT = real_type;
      using IdxT  = index_type;

      std::string device_class{"bus"};
      std::string name;
      IdxT        bus_id{0};

      std::array<RealT, N> v0{};
    };
  } // namespace EMT
} // namespace GridKit
