#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT = double, typename IdxT = std::size_t>
    using CaseData = SystemModelData<RealT,
                                     IdxT,
                                     BranchLumpedConstant<RealT, IdxT>,
                                     Breaker<RealT, IdxT>,
                                     LoadRL<RealT, IdxT>,
                                     VoltageSource<RealT, IdxT>>;

    template <class DataT>
    struct CaseNames
    {
      using IndexT = typename DataT::index_type;

      std::unordered_map<std::string, IndexT>       buses;
      std::unordered_map<std::string, ComponentRef> components;
      std::unordered_set<std::string>               names;

      IndexT bus(std::string_view name) const
      {
        const auto key = std::string(name);
        const auto it  = buses.find(key);
        if (it == buses.end())
        {
          throw CaseError("unknown bus '" + key + "'");
        }
        return it->second;
      }

      ComponentRef component(std::string_view name) const
      {
        const auto key = std::string(name);
        const auto it  = components.find(key);
        if (it == components.end())
        {
          throw CaseError("unknown component '" + key + "'");
        }
        return it->second;
      }
    };

    template <class DataT = CaseData<>>
    struct Case
    {
      DataT            data;
      CaseNames<DataT> names;
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/IO/CaseJson.hpp>
