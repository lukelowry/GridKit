#pragma once

#include <vector>

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit::EMT
{
  template <typename scalar_type, typename index_type>
  struct BusVoltageContribution
  {
    using RealT = typename ScalarTraits<scalar_type>::RealT;
    using IdxT  = index_type;

    IdxT             bus_id;
    ABCMatrix<RealT> derivative_block{};
    ABCMatrix<RealT> algebraic_block{};
  };

  template <typename scalar_type, typename index_type>
  class BusVoltageContributor
  {
  public:
    using ContributionT = BusVoltageContribution<scalar_type, index_type>;

    virtual ~BusVoltageContributor() = default;

    virtual void appendBusVoltageContributions(
        std::vector<ContributionT>& contributions) const = 0;
  };
} // namespace GridKit::EMT
