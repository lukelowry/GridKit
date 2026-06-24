#pragma once

#include <cstddef>

#include <GridKit/Model/PhasorDynamics/Component.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    class Component : public PhasorDynamics::Component<scalar_type, index_type>
    {
    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using BaseT   = PhasorDynamics::Component<ScalarT, IdxT>;
      using RealT   = typename BaseT::RealT;
      using MatrixT = typename BaseT::MatrixT;

      Component()           = default;
      ~Component() override = default;

    protected:
      template <typename BusT>
      void addCurrentInjection(BusT& bus, const ScalarT* i_inj)
      {
        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          bus.I(n) += i_inj[static_cast<std::size_t>(n)];
        }
      }
    };
  } // namespace EMT
} // namespace GridKit
