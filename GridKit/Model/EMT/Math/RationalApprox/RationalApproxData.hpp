#pragma once

#include <cstddef>
#include <vector>

namespace GridKit
{
  namespace EMT
  {
    namespace Math
    {
      template <typename RealT>
      struct RationalApproxData
      {
        std::size_t dimension{};

        std::vector<RealT> d;
        std::vector<RealT> e;

        std::vector<RealT> real_poles;
        std::vector<RealT> real_residues;

        std::vector<RealT> pair_real;
        std::vector<RealT> pair_imag;
        std::vector<RealT> pair_residue_real;
        std::vector<RealT> pair_residue_imag;
      };
    } // namespace Math
  } // namespace EMT
} // namespace GridKit
