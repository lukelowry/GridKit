#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/Math/RationalApprox/RationalApproxData.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Math
    {
      template <class ScalarT, typename IdxT = std::size_t>
      class RationalApprox
      {
      public:
        using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
        using DataT = RationalApproxData<RealT, IdxT>;

        RationalApprox() = default;
        explicit RationalApprox(const DataT& data);

        void         setData(const DataT& data);
        const DataT& data() const;

        IdxT dimension() const;
        IdxT matrixSize() const;
        IdxT realPoleCount() const;
        IdxT complexPairCount() const;
        IdxT stateCount() const;
        IdxT equationCount() const;

        bool hasDerivativeFeedthrough() const;
        int  verify() const;

        int initialize(const ScalarT* u0, const ScalarT* up0, ScalarT* x, ScalarT* xp) const;
        int evaluateStateResidual(const ScalarT* u, const ScalarT* x, const ScalarT* xp, ScalarT* f) const;
        int evaluateOutput(const ScalarT* u, const ScalarT* up, const ScalarT* x, ScalarT* z) const;

      private:
        DataT data_{};
      };
    } // namespace Math
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Math/RationalApprox/RationalApproxImpl.hpp>
