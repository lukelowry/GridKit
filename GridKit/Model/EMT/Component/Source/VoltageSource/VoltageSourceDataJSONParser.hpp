#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT, typename IdxT, std::size_t N>
    void from_json(const json& j, VoltageSourceData<RealT, IdxT, N>& data)
    {
      j.at("class").get_to(data.device_class);
      j.at("id").get_to(data.disambiguation_string);

      const auto& params = j.at("params");

      auto readArray = [](const json& values, std::array<RealT, N>& target, const std::string& name)
      {
        if (values.size() != N)
        {
          throw std::invalid_argument("VoltageSource: " + name + " has invalid length");
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          target[n] = values.at(n).template get<RealT>();
          if (!std::isfinite(target[n]))
          {
            throw std::invalid_argument("VoltageSource: " + name + " contains a non-finite value");
          }
        }
      };

      readArray(params.at("E"), data.E, "E");
      readArray(params.at("phi"), data.phi, "phi");
      readArray(params.at("G"), data.G, "G");

      params.at("omega").get_to(data.omega);
      if (!std::isfinite(data.omega) || data.omega <= RealT{0.0})
      {
        throw std::invalid_argument("VoltageSource: omega must be finite and positive");
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        if (data.E[n] < RealT{0.0})
        {
          throw std::invalid_argument("VoltageSource: E must be nonnegative");
        }

        if (data.G[n] <= RealT{0.0})
        {
          throw std::invalid_argument("VoltageSource: G must be positive");
        }
      }
    }
  } // namespace EMT
} // namespace GridKit
