#pragma once

#include <array>
#include <map>
#include <string>

namespace GridKit
{
  namespace EMT
  {
    enum class LoadRLPorts
    {
      bus
    };

    template <typename real_type, typename index_type>
    struct LoadRLData
    {
      using RealT = real_type;
      using IdxT  = index_type;
      using Ports = LoadRLPorts;

      LoadRLData()
      {
        L.fill(RealT{1.0});
      }

      std::string device_class{"LoadRL"};
      std::string disambiguation_string;

      std::map<Ports, IdxT> ports;

      std::array<RealT, 3> R{};
      std::array<RealT, 3> L{};
      RealT                Iinj{0.0};
      RealT                theta{0.0};
    };
  } // namespace EMT
} // namespace GridKit
