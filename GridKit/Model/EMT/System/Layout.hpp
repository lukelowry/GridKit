#pragma once

#include <string>
#include <vector>

#include <GridKit/Model/EMT/System/LocalMap.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace System
    {
      enum class ComponentKind
      {
        BranchLumpedConstant,
        LoadRL,
        VoltageSource
      };

      template <typename RealT, typename IdxT>
      struct BusBlock
      {
        IdxT        external_id{0};
        std::string name;
        IdxT        variable_start{0};
        IdxT        equation_start{0};
        RealT       vm{0.0};
        RealT       va{0.0};
        RealT       frequency{60.0};

        static constexpr IdxT width = 3;
      };

      template <typename IdxT>
      struct ComponentBlock
      {
        ComponentKind  kind;
        IdxT           component_index{0};
        IdxT           own_variable_start{0};
        IdxT           own_equation_start{0};
        LocalMap<IdxT> local_map;
      };

      template <typename RealT, typename IdxT>
      struct Layout
      {
        std::vector<BusBlock<RealT, IdxT>> buses;
        std::vector<ComponentBlock<IdxT>>  components;
        IdxT                               size{0};
      };
    } // namespace System
  } // namespace EMT
} // namespace GridKit
