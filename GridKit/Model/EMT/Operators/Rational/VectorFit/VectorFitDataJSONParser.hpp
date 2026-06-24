#pragma once

#include <stdexcept>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT, typename IdxT, std::size_t N, std::size_t K>
    void from_json(const json& j, VectorFitData<RealT, IdxT, N, K>& data)
    {
      j.at("class").get_to(data.device_class);
      j.at("id").get_to(data.disambiguation_string);

      const auto& params = j.at("params");

      for (std::size_t n = 0; n < N; ++n)
      {
        for (std::size_t k = 0; k < K; ++k)
        {
          data.D(n, k) = params.at("D").at(n).at(k).template get<RealT>();
          data.E(n, k) = params.at("E").at(n).at(k).template get<RealT>();
        }
      }

      const auto q_count = params.at("poles").size();
      if (params.at("A").size() != q_count || params.at("B").size() != q_count)
      {
        throw std::invalid_argument("VectorFit: pole and residue counts do not match");
      }

      data.poles.resize(q_count);
      data.A.resize(q_count);
      data.B.resize(q_count);

      for (std::size_t q = 0; q < q_count; ++q)
      {
        data.poles[q][0] = params.at("poles").at(q).at(0).template get<RealT>();
        data.poles[q][1] = params.at("poles").at(q).at(1).template get<RealT>();

        for (std::size_t n = 0; n < N; ++n)
        {
          for (std::size_t k = 0; k < K; ++k)
          {
            data.A[q](n, k) = params.at("A").at(q).at(n).at(k).template get<RealT>();
            data.B[q](n, k) = params.at("B").at(q).at(n).at(k).template get<RealT>();
          }
        }
      }
    }
  } // namespace EMT
} // namespace GridKit
