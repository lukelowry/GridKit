/**
 * @file LineData.hpp
 * @brief Data for EMT transmission line models.
 */

#pragma once

#include <GridKit/Model/EMT/RationalApprox/RationalApproxData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT, typename IdxT>
    struct LineData
    {
      RationalApproxData<RealT, IdxT> characteristic_admittance;
    };

  } // namespace EMT
} // namespace GridKit
