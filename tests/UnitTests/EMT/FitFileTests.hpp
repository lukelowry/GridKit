#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/IO/FitFile.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/Sha256.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class RealT>
    class EMTFitFileTests
    {
    public:
      using Json = nlohmann::json;

      TestOutcome validCharacteristicAdmittance()
      {
        TestStatus success = true;

        const auto dir  = std::filesystem::path("EMTFitFileTest");
        const auto file = dir / "valid.yc.fit.json";
        resetDir(dir);
        writeJson(file, validFitJson());

        const auto fit = EMT::IO::readCharacteristicAdmittanceFit<RealT>(
            EMT::IO::FitFileReference{file, fileSha256(file)},
            "fit");

        success *= (fit.format_version == 1);
        success *= (fit.fit_name == "unit_yc");
        success *= (fit.quantity == "characteristic_admittance");
        success *= (fit.phase_order == std::vector<std::string>{"a", "b", "c"});
        success *= (fit.data.dimension == 3u);
        success *= (fit.data.d.size() == 9u);
        success *= (fit.data.real_poles.size() == 1u);
        success *= (fit.data.real_residues.size() == 9u);

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome rejectsInvalidFiles()
      {
        TestStatus success = true;

        const auto dir = std::filesystem::path("EMTFitFileInvalidTest");
        resetDir(dir);

        const auto malformed = dir / "malformed.fit.json";
        {
          std::ofstream out(malformed);
          out << "{ bad json";
        }
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::IO::readCharacteristicAdmittanceFit<RealT>(
                  EMT::IO::FitFileReference{malformed, std::nullopt},
                  "fit");
            },
            "malformed JSON");

        success *= rejectedMutation(dir, "bad_version.fit.json", "unsupported version", [](Json& json)
                                    { json["format_version"] = 2; });
        success *= rejectedMutation(dir, "bad_schema.fit.json", "schema", [](Json& json)
                                    { json["schema"] = "gridkit.emt.other"; });
        success *= rejectedMutation(dir, "bad_quantity.fit.json", "characteristic_admittance", [](Json& json)
                                    { json["quantity"] = "propagation_function"; });
        success *= rejectedMutation(dir, "bad_domain.fit.json", "domain", [](Json& json)
                                    { json["domain"] = "modal"; });
        success *= rejectedMutation(dir, "bad_realization.fit.json", "realization", [](Json& json)
                                    { json["realization"] = "state_space"; });
        success *= rejectedMutation(dir, "bad_layout.fit.json", "matrix_layout", [](Json& json)
                                    { json["matrix_layout"] = "column_major"; });
        success *= rejectedMutation(dir, "bad_units.fit.json", "units.transfer", [](Json& json)
                                    { json["units"]["transfer"] = "ohm"; });
        success *= rejectedMutation(dir, "bad_s_units.fit.json", "s_units", [](Json& json)
                                    { json["transfer_function"]["s_units"] = "Hz"; });
        success *= rejectedMutation(dir, "bad_phase.fit.json", "phase_order", [](Json& json)
                                    { json["phase_order"] = Json::array({"a", "c", "b"}); });
        success *= rejectedMutation(dir, "bad_pole.fit.json", "strictly stable", [](Json& json)
                                    { json["rational_approx"]["real_poles"][0] = 1.0; });
        success *= rejectedNonFinite(dir);
        success *= rejectedMutation(dir, "bad_size.fit.json", "expected 9 entries", [](Json& json)
                                    { json["rational_approx"]["d"].push_back(0.0); });
        success *= rejectedMutation(dir, "bad_unknown.fit.json", "unknown key", [](Json& json)
                                    { json["surprise"] = 1; });
        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome pathAndHashValidation()
      {
        TestStatus success = true;

        const auto dir  = std::filesystem::path("EMTFitFileHashTest");
        const auto file = dir / "valid.yc.fit.json";
        resetDir(dir);
        writeJson(file, validFitJson());

        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::IO::readCharacteristicAdmittanceFit<RealT>(
                  EMT::IO::FitFileReference{file, std::string(64, '0')},
                  "fit");
            },
            "sha256 mismatch");

        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::IO::readCharacteristicAdmittanceFit<RealT>(
                  EMT::IO::FitFileReference{dir / "missing.fit.json", std::nullopt},
                  "fit");
            },
            "not a regular file");

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

    private:
      static Json validFitJson()
      {
        return Json{
            {"schema", "gridkit.emt.rational_fit"},
            {"format_version", 1},
            {"fit_name", "unit_yc"},
            {"quantity", "characteristic_admittance"},
            {"domain", "phase"},
            {"realization", "pole_residue_real_sections"},
            {"matrix_layout", "row_major"},
            {"phase_order", Json::array({"a", "b", "c"})},
            {"transfer_function",
             Json{{"variable", "s"}, {"s_units", "rad/s"}, {"form", "D + sE + sum(R/(s-p))"}}},
            {"units",
             Json{{"transfer", "S"}, {"d", "S"}, {"e", "S*s"}, {"poles", "rad/s"}, {"residues", "S/s"}}},
            {"source", Json::object()},
            {"rational_approx",
             Json{{"dimension", 3},
                  {"d", Json::array({1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0})},
                  {"e", Json::array({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
                  {"real_poles", Json::array({-10.0})},
                  {"real_residues", Json::array({0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3})},
                  {"pair_real", Json::array()},
                  {"pair_imag", Json::array()},
                  {"pair_residue_real", Json::array()},
                  {"pair_residue_imag", Json::array()}}},
            {"validation", Json::object()},
            {"metadata", Json::object()}};
      }

      static void resetDir(const std::filesystem::path& dir)
      {
        std::filesystem::remove_all(dir);
        std::filesystem::create_directory(dir);
      }

      static void writeJson(const std::filesystem::path& path, const Json& json)
      {
        std::ofstream out(path);
        out << json.dump(2) << '\n';
      }

      static std::string fileSha256(const std::filesystem::path& path)
      {
        std::ifstream input(path, std::ios::binary);
        const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(input),
                                              std::istreambuf_iterator<char>()};
        return GridKit::Utilities::sha256Hex(std::span<const std::uint8_t>{bytes.data(), bytes.size()});
      }

      template <class Mutator>
      static bool rejectedMutation(const std::filesystem::path& dir,
                                   const std::string&           name,
                                   std::string_view             text,
                                   Mutator&&                    mutator)
      {
        auto json = validFitJson();
        mutator(json);
        const auto file = dir / name;
        writeJson(file, json);
        return caseErrorContains(
            [&]()
            {
              (void) EMT::IO::readCharacteristicAdmittanceFit<RealT>(
                  EMT::IO::FitFileReference{file, std::nullopt},
                  "fit");
            },
            text);
      }

      static bool rejectedNonFinite(const std::filesystem::path& dir)
      {
        auto       text = validFitJson().dump(2);
        const auto pos  = text.find("1.0");
        if (pos == std::string::npos)
        {
          return false;
        }
        text.replace(pos, 3u, "1e999");

        const auto file = dir / "bad_nonfinite.fit.json";
        {
          std::ofstream out(file);
          out << text << '\n';
        }
        return caseErrorContainsAny(
            [&]()
            {
              (void) EMT::IO::readCharacteristicAdmittanceFit<RealT>(
                  EMT::IO::FitFileReference{file, std::nullopt},
                  "fit");
            },
            {"finite", "malformed JSON"});
      }

      template <class Fn>
      static bool caseErrorContains(Fn&& fn, std::string_view text)
      {
        try
        {
          fn();
        }
        catch (const EMT::CaseError& ex)
        {
          return std::string(ex.what()).find(text) != std::string::npos;
        }
        catch (...)
        {
          return false;
        }
        return false;
      }

      template <class Fn>
      static bool caseErrorContainsAny(Fn&& fn, std::initializer_list<std::string_view> texts)
      {
        try
        {
          fn();
        }
        catch (const EMT::CaseError& ex)
        {
          const std::string message = ex.what();
          for (const auto text : texts)
          {
            if (message.find(text) != std::string::npos)
            {
              return true;
            }
          }
          return false;
        }
        catch (...)
        {
          return false;
        }
        return false;
      }
    };
  } // namespace Testing
} // namespace GridKit
