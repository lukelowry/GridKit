#pragma once

#include <cmath>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRLData.hpp>
#include <GridKit/Model/EMT/ComponentDataJSONParser.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT, typename IdxT>
    void from_json(const json& j, LoadRLData<RealT, IdxT>& data)
    {
      readComponentData(j, data, "LoadRL");

      const auto& params = j.at("params");

      params.at("Ra").get_to(data.R[0]);
      params.at("Rb").get_to(data.R[1]);
      params.at("Rc").get_to(data.R[2]);
      params.at("La").get_to(data.L[0]);
      params.at("Lb").get_to(data.L[1]);
      params.at("Lc").get_to(data.L[2]);

      if (params.contains("Iinj"))
      {
        params.at("Iinj").get_to(data.Iinj);
      }
      if (params.contains("theta"))
      {
        params.at("theta").get_to(data.theta);
      }

      for (std::size_t n = 0; n < 3; ++n)
      {
        if (!std::isfinite(data.R[n]) || data.R[n] < RealT{0.0})
        {
          throw std::invalid_argument("LoadRL: R must be finite and nonnegative");
        }

        if (!std::isfinite(data.L[n]) || data.L[n] <= RealT{0.0})
        {
          throw std::invalid_argument("LoadRL: L must be finite and positive");
        }
      }

      if (!std::isfinite(data.Iinj) || data.Iinj < RealT{0.0})
      {
        throw std::invalid_argument("LoadRL: Iinj must be finite and nonnegative");
      }

      if (!std::isfinite(data.theta))
      {
        throw std::invalid_argument("LoadRL: theta must be finite");
      }
    }
  } // namespace EMT
} // namespace GridKit
