#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <GridKit/Model/PhasorDynamics/VectorizedEquations.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type, typename index_type, std::size_t N, std::size_t K>
    struct VectorFitData
    {
      using RealT   = real_type;
      using IdxT    = index_type;
      using MatrixT = PhasorDynamics::Equation::Mat<RealT, N, K>;
      using PoleT   = std::array<RealT, 2>;

      std::string device_class{"VectorFit"};
      std::string disambiguation_string;

      MatrixT D{};
      MatrixT E{};

      std::vector<PoleT>   poles{};
      std::vector<MatrixT> A{};
      std::vector<MatrixT> B{};
    };
  } // namespace EMT
} // namespace GridKit
