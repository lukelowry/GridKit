/**
 * @file PropagationDataJSONParser.hpp
 *
 * @brief JSON parser for composite EMT Propagation operator data.
 *
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpaceDataJSONParser.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Propagation/PropagationData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        namespace Detail
        {
          namespace fs = std::filesystem;

          using json = ::nlohmann::json;

          inline std::ifstream openPropagationJsonFile(const fs::path& file_path)
          {
            if (!fs::exists(file_path))
            {
              throw std::runtime_error("Propagation model file not found: " + file_path.string());
            }

            auto stream = std::ifstream(file_path);
            if (!stream)
            {
              throw std::runtime_error("Failed to open Propagation model file: "
                                       + file_path.string());
            }
            return stream;
          }

          inline fs::path resolvePath(const fs::path& base_file, const std::string& raw_path)
          {
            const fs::path path(raw_path);
            if (path.is_absolute())
            {
              return path;
            }
            return base_file.parent_path() / path;
          }

          inline void rejectUnsupportedPropagationFields(const json& j)
          {
            if (!j.is_object())
            {
              throw std::runtime_error("Propagation model must be a JSON object");
            }

            for (const auto& entry : j.items())
            {
              const auto& key = entry.key();
              if (key != "class" && key != "input" && key != "tau" && key != "dt_min"
                  && key != "output")
              {
                throw std::runtime_error("Propagation model contains unsupported field: " + key);
              }
            }
          }

          template <typename ScalarT>
          ScalarT parsePositiveFinite(const json& raw, const std::string& context)
          {
            const auto value = raw.template get<ScalarT>();
            if (!std::isfinite(static_cast<double>(value)) || value <= ScalarT{0.0})
            {
              throw std::runtime_error(context + " must be positive and finite");
            }
            return value;
          }

          template <typename ScalarT>
          std::vector<ScalarT> parseDelayArray(const json& raw, const std::string& context)
          {
            if (!raw.is_array() || raw.empty())
            {
              throw std::runtime_error(context + " must be a nonempty array");
            }

            std::vector<ScalarT> values;
            values.reserve(raw.size());
            for (size_t i = 0; i < raw.size(); ++i)
            {
              values.push_back(
                  parsePositiveFinite<ScalarT>(raw.at(i), context + "[" + std::to_string(i) + "]"));
            }
            return values;
          }

          template <typename ScalarT>
          std::vector<ScalarT> parseTau(const json& raw, const fs::path& base_file)
          {
            if (raw.is_array())
            {
              return parseDelayArray<ScalarT>(raw, "Propagation tau");
            }
            if (!raw.is_string())
            {
              throw std::runtime_error("Propagation tau must be an array or JSON file path");
            }

            const auto path = resolvePath(base_file, raw.template get<std::string>());
            const auto j    = json::parse(openPropagationJsonFile(path));
            return parseDelayArray<ScalarT>(j, path.string());
          }

          template <typename ScalarT, typename IdxT>
          void validatePropagationData(const PropagationData<ScalarT, IdxT>& data)
          {
            const auto modal_count = static_cast<size_t>(data.input.N);
            if (data.tau.size() != modal_count)
            {
              throw std::runtime_error(
                  "Propagation tau size must match input StateSpace output dimension");
            }
            if (data.output.N != data.input.K)
            {
              throw std::runtime_error(
                  "Propagation output StateSpace output dimension must match external dimension");
            }
            if (data.output.K != data.input.N)
            {
              throw std::runtime_error(
                  "Propagation output StateSpace input dimension must match modal dimension");
            }
          }

          template <typename ScalarT, typename IdxT>
          PropagationData<ScalarT, IdxT>
          parsePropagationDataJson(const json& j, const fs::path& file_path)
          {
            rejectUnsupportedPropagationFields(j);

            const auto model_class = j.at("class").template get<std::string>();
            if (model_class != "Propagation")
            {
              throw std::runtime_error("Unsupported propagation model class: " + model_class);
            }

            if (!j.at("input").is_string())
            {
              throw std::runtime_error("Propagation input must be a StateSpace model path");
            }
            if (!j.at("output").is_string())
            {
              throw std::runtime_error("Propagation output must be a StateSpace model path");
            }

            PropagationData<ScalarT, IdxT> data;
            data.input =
                Rational::parseStateSpaceData<ScalarT, IdxT>(
                    resolvePath(file_path, j.at("input").template get<std::string>()));
            data.tau    = parseTau<ScalarT>(j.at("tau"), file_path);
            data.dt_min = parsePositiveFinite<ScalarT>(j.at("dt_min"), "Propagation dt_min");
            data.output =
                Rational::parseStateSpaceData<ScalarT, IdxT>(
                    resolvePath(file_path, j.at("output").template get<std::string>()));

            validatePropagationData(data);
            return data;
          }
        } // namespace Detail

        template <typename ScalarT = double, typename IdxT = size_t>
        PropagationData<ScalarT, IdxT>
        parsePropagationData(const std::filesystem::path& file_path)
        {
          const auto j = Detail::json::parse(Detail::openPropagationJsonFile(file_path));
          return Detail::parsePropagationDataJson<ScalarT, IdxT>(j, file_path);
        }

      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
