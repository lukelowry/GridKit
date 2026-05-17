#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/EMT/Math/RationalApprox/RationalApprox.hpp>
#include <GridKit/Model/EMT/Math/RationalApprox/RationalApproxData.hpp>
#include <GridKit/Utilities/Sha256.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace IO
    {
      struct FitFileReference
      {
        std::filesystem::path     fit_file;
        std::optional<std::string> sha256;
      };

      template <class RealT>
      struct RationalFitFile
      {
        int                          format_version{1};
        std::filesystem::path        path;
        std::string                  fit_name;
        std::string                  quantity;
        std::string                  domain;
        std::string                  realization;
        std::string                  matrix_layout;
        std::string                  transfer_units;
        std::string                  s_units;
        std::vector<std::string>     phase_order;
        Math::RationalApproxData<RealT> data;
      };

      namespace Detail
      {
        using GridKit::EMT::Detail::fieldContext;
        using GridKit::EMT::Detail::readJsonValue;
        using GridKit::EMT::Detail::throwCase;

        inline std::filesystem::path resolveFitPath(const std::filesystem::path& base,
                                                    std::filesystem::path        path)
        {
          if (!path.empty() && !path.is_absolute() && !base.empty())
          {
            path = base / path;
          }
          return path;
        }

        inline std::string lowerAscii(std::string value)
        {
          std::transform(value.begin(),
                         value.end(),
                         value.begin(),
                         [](unsigned char ch)
                         {
                           return static_cast<char>(std::tolower(ch));
                         });
          return value;
        }

        inline bool isHexSha256(std::string_view value)
        {
          if (value.size() != 64u)
          {
            return false;
          }
          return std::all_of(value.begin(),
                             value.end(),
                             [](unsigned char ch)
                             {
                               return std::isxdigit(ch) != 0;
                             });
        }

        inline void requireStringEquals(const nlohmann::json& obj,
                                        std::string_view      key,
                                        std::string_view      expected,
                                        std::string_view      entity)
        {
          const auto value = require<std::string>(obj, key, entity);
          if (value != expected)
          {
            throw CaseError(std::string(fieldContext(entity, key)) + ": expected '"
                            + std::string(expected) + "'");
          }
        }

        inline const nlohmann::json& requireObject(const nlohmann::json& obj,
                                                   std::string_view      key,
                                                   std::string_view      entity)
        {
          if (!obj.is_object())
          {
            throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            throwCase(fieldContext(entity, key), "required field missing");
          }
          if (!it->is_object())
          {
            throwCase(fieldContext(entity, key), "expected object");
          }
          return *it;
        }

        inline void requireOptionalObject(const nlohmann::json& obj,
                                          std::string_view      key,
                                          std::string_view      entity)
        {
          const auto it = obj.find(std::string(key));
          if (it != obj.end() && !it->is_object())
          {
            throwCase(fieldContext(entity, key), "expected object");
          }
        }

        inline std::vector<std::uint8_t> readFileBytes(const std::filesystem::path& path,
                                                       std::string_view             entity)
        {
          std::error_code ec;
          if (!std::filesystem::is_regular_file(path, ec))
          {
            throw CaseError(std::string(entity) + ": fit file '" + path.string()
                            + "' is not a regular file");
          }

          std::ifstream input(path, std::ios::binary);
          if (!input)
          {
            throw CaseError(std::string(entity) + ": fit file '" + path.string()
                            + "' could not be opened");
          }
          return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        }

        inline nlohmann::json parseJsonBytes(std::span<const std::uint8_t> bytes,
                                             const std::filesystem::path&   path,
                                             std::string_view              entity)
        {
          try
          {
            return nlohmann::json::parse(bytes.begin(), bytes.end());
          }
          catch (const nlohmann::json::parse_error& ex)
          {
            throw CaseError(std::string(entity) + ": fit file '" + path.string()
                            + "': malformed JSON: " + std::string(ex.what()));
          }
          catch (const nlohmann::json::exception& ex)
          {
            throw CaseError(std::string(entity) + ": fit file '" + path.string()
                            + "': malformed JSON: " + std::string(ex.what()));
          }
        }

        template <class RealT>
        RealT readFiniteNumber(const nlohmann::json& value, std::string_view entity)
        {
          if (value.is_array() || value.is_object())
          {
            throwCase(entity, "expected number");
          }
          RealT result{};
          try
          {
            result = value.get<RealT>();
          }
          catch (const nlohmann::json::exception& ex)
          {
            throwCase(entity, ex.what());
          }
          if (!std::isfinite(result))
          {
            throwCase(entity, "must be finite");
          }
          return result;
        }

        template <class RealT>
        std::vector<RealT> readFiniteVector(const nlohmann::json& obj,
                                            std::string_view      key,
                                            std::string_view      entity)
        {
          if (!obj.is_object())
          {
            throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            throwCase(fieldContext(entity, key), "required field missing");
          }
          if (!it->is_array())
          {
            throwCase(fieldContext(entity, key), "expected array");
          }

          std::vector<RealT> result;
          result.reserve(it->size());
          for (std::size_t i = 0; i < it->size(); ++i)
          {
            result.push_back(readFiniteNumber<RealT>((*it)[i],
                                                     std::string(fieldContext(entity, key))
                                                         + "[" + std::to_string(i) + "]"));
          }
          return result;
        }

        inline std::size_t readPositiveSize(const nlohmann::json& obj,
                                            std::string_view      key,
                                            std::string_view      entity)
        {
          if (!obj.is_object())
          {
            throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            throwCase(fieldContext(entity, key), "required field missing");
          }
          if (!it->is_number_integer())
          {
            throwCase(fieldContext(entity, key), "expected positive integer");
          }
          const auto raw = it->get<long long>();
          if (raw <= 0)
          {
            throwCase(fieldContext(entity, key), "must be positive");
          }
          return static_cast<std::size_t>(raw);
        }

        inline std::vector<std::string> readStringArray(const nlohmann::json& obj,
                                                        std::string_view      key,
                                                        std::string_view      entity)
        {
          if (!obj.is_object())
          {
            throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            throwCase(fieldContext(entity, key), "required field missing");
          }
          if (!it->is_array())
          {
            throwCase(fieldContext(entity, key), "expected array");
          }

          std::vector<std::string> result;
          result.reserve(it->size());
          for (std::size_t i = 0; i < it->size(); ++i)
          {
            result.push_back(readJsonValue<std::string>((*it)[i],
                                                        std::string(fieldContext(entity, key))
                                                            + "[" + std::to_string(i) + "]"));
          }
          return result;
        }

        template <class VectorT>
        void requireSize(const VectorT& vector, std::size_t expected, std::string_view entity)
        {
          if (vector.size() != expected)
          {
            throw CaseError(std::string(entity) + ": expected " + std::to_string(expected)
                            + " entries, got " + std::to_string(vector.size()));
          }
        }

        template <class RealT>
        Math::RationalApproxData<RealT> readRationalApproxData(const nlohmann::json& obj,
                                                               std::string_view      entity)
        {
          if (!obj.is_object())
          {
            throwCase(entity, "expected object");
          }
          rejectUnknownKeys(obj,
                            {"dimension",
                             "d",
                             "e",
                             "real_poles",
                             "real_residues",
                             "pair_real",
                             "pair_imag",
                             "pair_residue_real",
                             "pair_residue_imag"},
                            entity);

          Math::RationalApproxData<RealT> data;
          data.dimension          = readPositiveSize(obj, "dimension", entity);
          data.d                  = readFiniteVector<RealT>(obj, "d", entity);
          data.e                  = readFiniteVector<RealT>(obj, "e", entity);
          data.real_poles         = readFiniteVector<RealT>(obj, "real_poles", entity);
          data.real_residues      = readFiniteVector<RealT>(obj, "real_residues", entity);
          data.pair_real          = readFiniteVector<RealT>(obj, "pair_real", entity);
          data.pair_imag          = readFiniteVector<RealT>(obj, "pair_imag", entity);
          data.pair_residue_real  = readFiniteVector<RealT>(obj, "pair_residue_real", entity);
          data.pair_residue_imag  = readFiniteVector<RealT>(obj, "pair_residue_imag", entity);

          const std::size_t n2 = data.dimension * data.dimension;
          requireSize(data.d, n2, fieldContext(entity, "d"));
          requireSize(data.e, n2, fieldContext(entity, "e"));
          requireSize(data.real_residues,
                      data.real_poles.size() * n2,
                      fieldContext(entity, "real_residues"));
          requireSize(data.pair_real, data.pair_imag.size(), fieldContext(entity, "pair_real"));
          requireSize(data.pair_residue_real,
                      data.pair_real.size() * n2,
                      fieldContext(entity, "pair_residue_real"));
          requireSize(data.pair_residue_imag,
                      data.pair_real.size() * n2,
                      fieldContext(entity, "pair_residue_imag"));

          for (std::size_t i = 0; i < data.real_poles.size(); ++i)
          {
            if (data.real_poles[i] >= RealT{0.0})
            {
              throwCase(std::string(fieldContext(entity, "real_poles")) + "["
                            + std::to_string(i) + "]",
                        "must be strictly stable");
            }
          }
          for (std::size_t i = 0; i < data.pair_real.size(); ++i)
          {
            if (data.pair_real[i] >= RealT{0.0})
            {
              throwCase(std::string(fieldContext(entity, "pair_real")) + "["
                            + std::to_string(i) + "]",
                        "must be strictly stable");
            }
            if (data.pair_imag[i] <= RealT{0.0})
            {
              throwCase(std::string(fieldContext(entity, "pair_imag")) + "["
                            + std::to_string(i) + "]",
                        "must be positive");
            }
          }

          try
          {
            (void) Math::RationalApprox<RealT>(data);
          }
          catch (const std::exception& ex)
          {
            throw CaseError(std::string(entity) + ": " + ex.what());
          }
          return data;
        }
      } // namespace Detail

      inline FitFileReference readFitFileReference(const nlohmann::json&        obj,
                                                   const std::filesystem::path& base_dir,
                                                   std::string_view             entity)
      {
        if (!obj.is_object())
        {
          Detail::throwCase(entity, "expected object");
        }
        rejectUnknownKeys(obj, {"fit_file", "sha256"}, entity);

        const auto raw_path = require<std::string>(obj, "fit_file", entity);
        if (raw_path.empty())
        {
          Detail::throwCase(Detail::fieldContext(entity, "fit_file"), "must not be empty");
        }

        FitFileReference ref;
        ref.fit_file = Detail::resolveFitPath(base_dir, std::filesystem::path{raw_path});

        const auto hash_it = obj.find("sha256");
        if (hash_it != obj.end())
        {
          auto hash = Detail::lowerAscii(Detail::readJsonValue<std::string>(*hash_it,
                                                                            Detail::fieldContext(entity, "sha256")));
          if (!Detail::isHexSha256(hash))
          {
            Detail::throwCase(Detail::fieldContext(entity, "sha256"),
                              "expected 64 hexadecimal characters");
          }
          ref.sha256 = std::move(hash);
        }
        return ref;
      }

      template <class RealT>
      RationalFitFile<RealT> readFitFile(const FitFileReference& ref,
                                         std::string_view        entity)
      {
        const auto bytes = Detail::readFileBytes(ref.fit_file, entity);
        const auto hash  = GridKit::Utilities::sha256Hex(
            std::span<const std::uint8_t>{bytes.data(), bytes.size()});
        if (ref.sha256.has_value() && Detail::lowerAscii(*ref.sha256) != hash)
        {
          throw CaseError(std::string(entity) + ": fit file sha256 mismatch for '"
                          + ref.fit_file.string() + "'");
        }

        const auto root = Detail::parseJsonBytes(
            std::span<const std::uint8_t>{bytes.data(), bytes.size()},
            ref.fit_file,
            entity);
        if (!root.is_object())
        {
          Detail::throwCase(entity, "fit file root must be an object");
        }
        rejectUnknownKeys(root,
                          {"schema",
                           "format_version",
                           "fit_name",
                           "quantity",
                           "domain",
                           "realization",
                           "matrix_layout",
                           "phase_order",
                           "transfer_function",
                           "units",
                           "source",
                           "rational_approx",
                           "validation",
                           "metadata"},
                          entity);

        Detail::requireStringEquals(root, "schema", "gridkit.emt.rational_fit", entity);

        RationalFitFile<RealT> fit;
        fit.path           = ref.fit_file;
        fit.format_version = require<int>(root, "format_version", entity);
        if (fit.format_version != 1)
        {
          throw CaseError(std::string(Detail::fieldContext(entity, "format_version"))
                          + ": unsupported version " + std::to_string(fit.format_version));
        }
        fit.fit_name      = require<std::string>(root, "fit_name", entity);
        fit.quantity      = require<std::string>(root, "quantity", entity);
        fit.domain        = require<std::string>(root, "domain", entity);
        fit.realization   = require<std::string>(root, "realization", entity);
        fit.matrix_layout = require<std::string>(root, "matrix_layout", entity);
        fit.phase_order   = Detail::readStringArray(root, "phase_order", entity);

        const auto& transfer = Detail::requireObject(root, "transfer_function", entity);
        rejectUnknownKeys(transfer, {"variable", "s_units", "form"}, Detail::fieldContext(entity, "transfer_function"));
        Detail::requireStringEquals(transfer,
                                    "variable",
                                    "s",
                                    Detail::fieldContext(entity, "transfer_function"));
        fit.s_units = require<std::string>(transfer,
                                           "s_units",
                                           Detail::fieldContext(entity, "transfer_function"));
        (void) require<std::string>(transfer,
                                    "form",
                                    Detail::fieldContext(entity, "transfer_function"));

        const auto& units = Detail::requireObject(root, "units", entity);
        rejectUnknownKeys(units, {"transfer", "d", "e", "poles", "residues"}, Detail::fieldContext(entity, "units"));
        fit.transfer_units = require<std::string>(units,
                                                  "transfer",
                                                  Detail::fieldContext(entity, "units"));
        Detail::requireStringEquals(units, "d", "S", Detail::fieldContext(entity, "units"));
        Detail::requireStringEquals(units, "e", "S*s", Detail::fieldContext(entity, "units"));
        Detail::requireStringEquals(units, "poles", "rad/s", Detail::fieldContext(entity, "units"));
        Detail::requireStringEquals(units, "residues", "S/s", Detail::fieldContext(entity, "units"));

        Detail::requireOptionalObject(root, "source", entity);
        Detail::requireOptionalObject(root, "validation", entity);
        Detail::requireOptionalObject(root, "metadata", entity);

        fit.data = Detail::readRationalApproxData<RealT>(
            Detail::requireObject(root, "rational_approx", entity),
            Detail::fieldContext(entity, "rational_approx"));

        return fit;
      }

      template <class RealT>
      RationalFitFile<RealT> readFitFile(const std::filesystem::path& path,
                                         std::string_view             entity)
      {
        return readFitFile<RealT>(FitFileReference{path, std::nullopt}, entity);
      }

      template <class RealT>
      RationalFitFile<RealT> readCharacteristicAdmittanceFit(const FitFileReference& ref,
                                                             std::string_view        entity)
      {
        auto fit = readFitFile<RealT>(ref, entity);
        if (fit.quantity != "characteristic_admittance")
        {
          throw CaseError(std::string(entity) + ".quantity: expected 'characteristic_admittance'");
        }
        if (fit.domain != "phase")
        {
          throw CaseError(std::string(entity) + ".domain: expected 'phase'");
        }
        if (fit.realization != "pole_residue_real_sections")
        {
          throw CaseError(std::string(entity) + ".realization: expected 'pole_residue_real_sections'");
        }
        if (fit.matrix_layout != "row_major")
        {
          throw CaseError(std::string(entity) + ".matrix_layout: expected 'row_major'");
        }
        if (fit.transfer_units != "S")
        {
          throw CaseError(std::string(entity) + ".units.transfer: expected 'S'");
        }
        if (fit.s_units != "rad/s")
        {
          throw CaseError(std::string(entity) + ".transfer_function.s_units: expected 'rad/s'");
        }
        if (fit.data.dimension != 3u)
        {
          throw CaseError(std::string(entity) + ".rational_approx.dimension: expected 3");
        }
        if (fit.phase_order != std::vector<std::string>{"a", "b", "c"})
        {
          throw CaseError(std::string(entity) + ".phase_order: expected ['a','b','c']");
        }
        return fit;
      }
    } // namespace IO
  } // namespace EMT
} // namespace GridKit
