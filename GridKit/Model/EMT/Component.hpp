#pragma once

#include <vector>

#include <GridKit/Model/EMT/GridElement.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class Component : public GridElement<ScalarT, IdxT>
    {
    public:
      using RealT   = typename GridElement<ScalarT, IdxT>::RealT;
      using MatrixT = typename GridElement<ScalarT, IdxT>::MatrixT;

      Component() = default;

      virtual ~Component()
      {
      }

      bool hasJacobian() override
      {
        return true;
      }

      void updateTime(RealT t, RealT a) override
      {
        time_  = t;
        alpha_ = a;
      }

      virtual int setGridKitComponentID(IdxT) = 0;

      IdxT getGridKitComponentID() const
      {
        return gridkit_component_id_;
      }

    protected:
      IdxT gridkit_component_id_{0};

      std::vector<ScalarT> wb_;
      std::vector<ScalarT> h_;

      RealT time_{0.0};
      RealT alpha_{0.0};
    };
  } // namespace EMT
} // namespace GridKit
