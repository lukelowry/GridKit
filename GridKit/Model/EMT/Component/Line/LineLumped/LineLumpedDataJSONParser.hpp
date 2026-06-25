#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitDataJSONParser.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT, typename IdxT, std::size_t N>
    void from_json(const json& j, LineLumpedData<RealT, IdxT, N>& data)
    {
      j.at("class").get_to(data.device_class);
      j.at("id").get_to(data.disambiguation_string);

      const auto& params = j.at("params");
      params.at("dx").get_to(data.dx);
      if (!std::isfinite(data.dx) || data.dx <= RealT{0.0})
      {
        throw std::invalid_argument("LineLumped: dx must be finite and positive");
      }

      if (params.contains("i0"))
      {
        for (std::size_t n = 0; n < N; ++n)
        {
          data.i0[n] = params.at("i0").at(n).template get<RealT>();
          if (!std::isfinite(data.i0[n]))
          {
            throw std::invalid_argument("LineLumped: i0 contains a non-finite value");
          }
        }
      }

      const bool has_fitted   = params.contains("Zp") || params.contains("Yp");
      const bool has_constant = params.contains("Rp") || params.contains("Lp")
                                || params.contains("Gp") || params.contains("Cp");

      if (has_fitted == has_constant)
      {
        throw std::invalid_argument("LineLumped: specify either Zp/Yp or Rp/Lp/Gp/Cp");
      }

      auto validateMatrix = [](const json& matrix, const std::string& name)
      {
        for (std::size_t r = 0; r < N; ++r)
        {
          for (std::size_t c = 0; c < N; ++c)
          {
            const RealT value = matrix.at(r).at(c).template get<RealT>();
            if (!std::isfinite(value))
            {
              throw std::invalid_argument("LineLumped: " + name + " contains a non-finite value");
            }
          }
        }
      };

      auto validateVectorFitData = [](const typename LineLumpedData<RealT, IdxT, N>::VectorFitDataT& fit,
                                      const std::string&                                             name)
      {
        for (std::size_t r = 0; r < N; ++r)
        {
          for (std::size_t c = 0; c < N; ++c)
          {
            if (!std::isfinite(fit.D(r, c)) || !std::isfinite(fit.E(r, c)))
            {
              throw std::invalid_argument("LineLumped: " + name + " contains a non-finite D or E value");
            }
          }
        }

        if (fit.A.size() != fit.poles.size() || fit.B.size() != fit.poles.size())
        {
          throw std::invalid_argument("LineLumped: " + name + " pole and residue counts do not match");
        }

        for (std::size_t q = 0; q < fit.poles.size(); ++q)
        {
          const RealT a     = fit.poles[q][0];
          const RealT omega = fit.poles[q][1];
          if (!std::isfinite(a) || !std::isfinite(omega) || a * a + omega * omega == RealT{0.0})
          {
            throw std::invalid_argument("LineLumped: " + name + " contains an invalid pole");
          }

          for (std::size_t r = 0; r < N; ++r)
          {
            for (std::size_t c = 0; c < N; ++c)
            {
              if (!std::isfinite(fit.A[q](r, c)) || !std::isfinite(fit.B[q](r, c)))
              {
                throw std::invalid_argument("LineLumped: " + name + " contains a non-finite residue value");
              }
            }
          }
        }
      };

      if (has_constant)
      {
        if (!params.contains("Rp") || !params.contains("Lp") || !params.contains("Gp") || !params.contains("Cp"))
        {
          throw std::invalid_argument("LineLumped: Rp, Lp, Gp, and Cp must be supplied together");
        }

        validateMatrix(params.at("Rp"), "Rp");
        validateMatrix(params.at("Lp"), "Lp");
        validateMatrix(params.at("Gp"), "Gp");
        validateMatrix(params.at("Cp"), "Cp");

        data.Zp.device_class          = "VectorFit";
        data.Zp.disambiguation_string = data.disambiguation_string + ".Zp";
        data.Yp.device_class          = "VectorFit";
        data.Yp.disambiguation_string = data.disambiguation_string + ".Yp";

        for (std::size_t r = 0; r < N; ++r)
        {
          for (std::size_t c = 0; c < N; ++c)
          {
            data.Zp.D(r, c) = params.at("Rp").at(r).at(c).template get<RealT>();
            data.Zp.E(r, c) = params.at("Lp").at(r).at(c).template get<RealT>();
            data.Yp.D(r, c) = params.at("Gp").at(r).at(c).template get<RealT>();
            data.Yp.E(r, c) = params.at("Cp").at(r).at(c).template get<RealT>();
          }
        }
      }
      else
      {
        if (!params.contains("Zp") || !params.contains("Yp"))
        {
          throw std::invalid_argument("LineLumped: Zp and Yp must be supplied together");
        }

        auto readVectorFit = [&](const char* key)
        {
          json raw = params.at(key);
          if (!raw.contains("class"))
          {
            raw["class"] = "VectorFit";
          }
          if (!raw.contains("id"))
          {
            raw["id"] = data.disambiguation_string + "." + key;
          }
          return raw.template get<typename LineLumpedData<RealT, IdxT, N>::VectorFitDataT>();
        };

        data.Zp = readVectorFit("Zp");
        data.Yp = readVectorFit("Yp");
      }

      validateVectorFitData(data.Zp, "Zp");
      validateVectorFitData(data.Yp, "Yp");
    }
  } // namespace EMT
} // namespace GridKit
