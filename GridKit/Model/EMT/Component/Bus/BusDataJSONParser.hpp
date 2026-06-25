#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Bus/BusData.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT, typename IdxT, std::size_t N>
    void from_json(const json& j, BusData<RealT, IdxT, N>& data)
    {
      j.at("number").get_to(data.bus_id);
      j.at("class").get_to(data.device_class);

      if (data.device_class != "bus")
      {
        throw std::invalid_argument("Bus: invalid class \"" + data.device_class + "\"");
      }

      data.name = j.value("name", std::string{});

      const auto& v0 = j.at("init").at("v0");
      if (!v0.is_array() || v0.size() != N)
      {
        throw std::invalid_argument("Bus: v0 has invalid length");
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        data.v0[n] = v0.at(n).template get<RealT>();
        if (!std::isfinite(data.v0[n]))
        {
          throw std::invalid_argument("Bus: initial voltage contains a non-finite value");
        }
      }
    }
  } // namespace EMT
} // namespace GridKit
