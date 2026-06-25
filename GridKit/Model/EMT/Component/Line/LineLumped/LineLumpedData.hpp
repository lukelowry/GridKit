#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <string>

#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LineLumpedPorts
    {
      bus1,
      bus2
    };

    template <typename real_type, typename index_type, std::size_t N>
    struct LineLumpedData
    {
      using RealT          = real_type;
      using IdxT           = index_type;
      using Ports          = LineLumpedPorts;
      using VectorFitDataT = VectorFitData<RealT, IdxT, N, N>;

      std::string device_class{"LineLumped"};
      std::string disambiguation_string;

      std::map<Ports, IdxT> ports;

      RealT                dx{1.0};
      std::array<RealT, N> i0{};
      VectorFitDataT       Zp{};
      VectorFitDataT       Yp{};
    };
  } // namespace EMT
} // namespace GridKit
