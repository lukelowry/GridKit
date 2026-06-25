#pragma once

#include <array>
#include <cstddef>
#include <string>

#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type, typename index_type, std::size_t N>
    struct LineLumpedData
    {
      using RealT          = real_type;
      using IdxT           = index_type;
      using VectorFitDataT = VectorFitData<RealT, IdxT, N, N>;

      std::string device_class{"LineLumped"};
      std::string disambiguation_string;

      RealT                dx{1.0};
      std::array<RealT, N> i0{};
      VectorFitDataT       Zp{};
      VectorFitDataT       Yp{};
    };
  } // namespace EMT
} // namespace GridKit
