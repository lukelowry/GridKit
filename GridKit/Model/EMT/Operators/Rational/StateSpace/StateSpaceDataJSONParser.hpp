/**
 * @file StateSpaceDataJSONParser.hpp
 *
 * @brief JSON parser for fitted rational state-space operator data.
 *
 */

#pragma once

#include <cmath>
#include <complex>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpaceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Rational
      {
        namespace Detail
        {
          namespace fs = std::filesystem;

          using json = ::nlohmann::json;

          inline std::ifstream openStateSpaceJsonFile(const fs::path& file_path)
          {
            if (!fs::exists(file_path))
            {
              throw std::runtime_error("StateSpace model file not found: " + file_path.string());
            }

            auto stream = std::ifstream(file_path);
            if (!stream)
            {
              throw std::runtime_error("Failed to open StateSpace model file: "
                                       + file_path.string());
            }
            return stream;
          }

          template <typename ScalarT>
          bool finite(ScalarT value)
          {
            return std::isfinite(static_cast<double>(value));
          }

          template <typename IdxT>
          IdxT parsePositiveIndex(const json& raw, const std::string& context)
          {
            unsigned long long value = 0;
            if (raw.is_number_unsigned())
            {
              value = raw.template get<unsigned long long>();
            }
            else if (raw.is_number_integer())
            {
              const auto signed_value = raw.template get<long long>();
              if (signed_value <= 0)
              {
                throw std::runtime_error(context + " must be positive");
              }
              value = static_cast<unsigned long long>(signed_value);
            }
            else
            {
              throw std::runtime_error(context + " must be an integer");
            }

            if (value == 0
                || value > static_cast<unsigned long long>(std::numeric_limits<IdxT>::max()))
            {
              throw std::runtime_error(context + " is not representable");
            }
            return static_cast<IdxT>(value);
          }

          inline size_t checkedSize(size_t rows, size_t cols, const std::string& context)
          {
            if (rows != 0 && cols > std::numeric_limits<size_t>::max() / rows)
            {
              throw std::runtime_error(context + " size is not representable");
            }
            return rows * cols;
          }

          inline void rejectUnsupportedStateSpaceFields(const json& j, const std::string& context)
          {
            if (!j.is_object())
            {
              throw std::runtime_error(context + " must be a JSON object");
            }

            for (const auto& entry : j.items())
            {
              const auto& key = entry.key();
              if (key == "poles" || key == "C" || key == "B" || key == "shape"
                  || key == "layout" || key == "d" || key == "e" || key == "rmse"
                  || key == "iters")
              {
                continue;
              }
              if (key == "residues")
              {
                throw std::runtime_error(context
                                         + " requires C/B state-space factors, not residues");
              }
              throw std::runtime_error(context + " contains unsupported field: " + key);
            }
          }

          template <typename ScalarT>
          ScalarT parseFiniteScalar(const json& raw, const std::string& context)
          {
            const auto value = raw.template get<ScalarT>();
            if (!finite(value))
            {
              throw std::runtime_error(context + " must be finite");
            }
            return value;
          }

          template <typename ScalarT>
          std::complex<ScalarT> parseComplexPair(const json& raw, const std::string& context)
          {
            if (!raw.is_array() || raw.size() != 2)
            {
              throw std::runtime_error(context + " must be [real, imag]");
            }

            return {parseFiniteScalar<ScalarT>(raw.at(0), context + " real part"),
                    parseFiniteScalar<ScalarT>(raw.at(1), context + " imaginary part")};
          }

          template <typename ScalarT>
          ScalarT parseRealComplexPair(const json& raw, const std::string& context)
          {
            const auto value = parseComplexPair<ScalarT>(raw, context);
            const auto scale = ScalarT{1.0} + std::abs(value.real());
            const auto tol =
                static_cast<ScalarT>(100.0) * std::numeric_limits<ScalarT>::epsilon() * scale;
            if (std::abs(value.imag()) > tol)
            {
              throw std::runtime_error(context + " must be real-valued");
            }
            return value.real();
          }

          template <typename ScalarT>
          std::vector<ScalarT> parseRealMatrixTerm(const json&        j,
                                                   const char*        key,
                                                   size_t             expected_size,
                                                   const std::string& context)
          {
            std::vector<ScalarT> values(expected_size, ScalarT{0.0});
            if (!j.contains(key))
            {
              return values;
            }

            const auto& raw = j.at(key);
            if (!raw.is_array() || raw.size() != expected_size)
            {
              throw std::runtime_error(context + ": \"" + key + "\" size must be N x K");
            }

            for (size_t i = 0; i < raw.size(); ++i)
            {
              values[i] =
                  parseRealComplexPair<ScalarT>(raw.at(i),
                                                context + ": \"" + key + "\"["
                                                    + std::to_string(i) + "]");
            }
            return values;
          }

          template <typename ScalarT>
          std::vector<std::complex<ScalarT>>
          parseComplexVector(const json& raw, size_t expected_size, const std::string& context)
          {
            if (!raw.is_array() || raw.size() != expected_size)
            {
              throw std::runtime_error(context + " size is invalid");
            }

            std::vector<std::complex<ScalarT>> values;
            values.reserve(expected_size);
            for (size_t i = 0; i < raw.size(); ++i)
            {
              values.push_back(
                  parseComplexPair<ScalarT>(raw.at(i), context + "[" + std::to_string(i) + "]"));
            }
            return values;
          }

          template <typename ScalarT>
          std::vector<std::complex<ScalarT>>
          parseComplexMatrix(const json&        raw,
                             size_t             rows,
                             size_t             cols,
                             const std::string& context)
          {
            if (!raw.is_array() || raw.size() != rows)
            {
              throw std::runtime_error(context + " row count is invalid");
            }

            std::vector<std::complex<ScalarT>> values;
            values.reserve(checkedSize(rows, cols, context));
            for (size_t i = 0; i < rows; ++i)
            {
              const auto& row = raw.at(i);
              if (!row.is_array() || row.size() != cols)
              {
                throw std::runtime_error(context + " column count is invalid");
              }
              for (size_t k = 0; k < cols; ++k)
              {
                values.push_back(parseComplexPair<ScalarT>(
                    row.at(k), context + "[" + std::to_string(i) + "][" + std::to_string(k) + "]"));
              }
            }
            return values;
          }

          template <typename ScalarT, typename IdxT>
          StateSpaceData<ScalarT, IdxT>
          parseStateSpaceDataJson(const json& j, const std::string& context)
          {
            rejectUnsupportedStateSpaceFields(j, context);

            if (j.contains("layout") && j.at("layout").template get<std::string>() != "RowMajor")
            {
              throw std::runtime_error(context + " only supports RowMajor layout");
            }

            const auto& raw_shape = j.at("shape");
            if (!raw_shape.is_array() || raw_shape.size() != 2)
            {
              throw std::runtime_error(context + " requires shape [N, K]");
            }

            StateSpaceData<ScalarT, IdxT> data;
            data.N = parsePositiveIndex<IdxT>(raw_shape.at(0), context + " shape[0]");
            data.K = parsePositiveIndex<IdxT>(raw_shape.at(1), context + " shape[1]");

            const auto N  = static_cast<size_t>(data.N);
            const auto K  = static_cast<size_t>(data.K);
            const auto NK = checkedSize(N, K, context + " N x K");

            const auto& raw_poles = j.at("poles");
            if (!raw_poles.is_array() || raw_poles.empty())
            {
              throw std::runtime_error(context + " requires a nonempty poles array");
            }
            if (raw_poles.size() > static_cast<size_t>(std::numeric_limits<IdxT>::max()))
            {
              throw std::runtime_error(context + " pole count is not representable");
            }

            data.Q     = static_cast<IdxT>(raw_poles.size());
            data.poles = parseComplexVector<ScalarT>(raw_poles, raw_poles.size(), context + " poles");

            const auto Q  = static_cast<size_t>(data.Q);
            const auto NQ = checkedSize(N, Q, context + " N x Q");
            const auto QK = checkedSize(Q, K, context + " Q x K");

            data.C = parseComplexMatrix<ScalarT>(j.at("C"), N, Q, context + " C");
            data.B = parseComplexMatrix<ScalarT>(j.at("B"), Q, K, context + " B");
            if (data.C.size() != NQ || data.B.size() != QK)
            {
              throw std::runtime_error(context + " C/B sizes are invalid");
            }

            data.D = parseRealMatrixTerm<ScalarT>(j, "d", NK, context);
            data.E = parseRealMatrixTerm<ScalarT>(j, "e", NK, context);

            return data;
          }
        } // namespace Detail

        template <typename ScalarT = double, typename IdxT = size_t>
        StateSpaceData<ScalarT, IdxT>
        parseStateSpaceData(const std::filesystem::path& file_path)
        {
          const auto j = Detail::json::parse(Detail::openStateSpaceJsonFile(file_path));
          return Detail::parseStateSpaceDataJson<ScalarT, IdxT>(j, file_path.string());
        }

      } // namespace Rational
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
