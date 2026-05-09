#pragma once

#include <stdexcept>
#include <string>

#include <GridKit/Model/Case/CaseData.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    class CaseDataTests
    {
    public:
      TestOutcome parsesMinimalCase()
      {
        TestStatus success = true;

        const auto data = Model::Case::parseCaseDataFromString(minimalCase());

        success *= (data.header.case_name == "minimal");
        success *= (data.buses.size() == 1);
        success *= (data.buses[0].number == 1);
        success *= (data.buses[0].component_class == "emt_bus");
        success *= data.branches.empty();
        success *= data.devices.empty();
        success *= data.signals.empty();

        return success.report(__func__);
      }

      TestOutcome requiresRootArrays()
      {
        TestStatus success = true;

        success *= Model::Case::parseCaseDataFromString(minimalCase()).signals.empty();
        success *= throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(withoutSignalsCase()); });

        return success.report(__func__);
      }

      TestOutcome rejectsUnknownFields()
      {
        TestStatus success = true;

        auto input = minimalCase();
        input.replace(input.find(R"("extension": {})"), std::string(R"("extension": {})").size(), R"("extension": {}, "extra": 1)");
        success *= throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(input); });

        auto extension_input = minimalCase();
        extension_input.replace(extension_input.find(R"("extension": {})"),
                                std::string(R"("extension": {})").size(),
                                R"("extension": {"extra": 1})");
        success *= !throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(extension_input); });

        return success.report(__func__);
      }

      TestOutcome rejectsDuplicateIds()
      {
        TestStatus success = true;

        success *= throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(duplicateBusCase()); });
        success *= throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(duplicateDeviceCase()); });
        success *= throws<std::runtime_error>(
            [&]()
            { Model::Case::parseCaseDataFromString(duplicateModelCase()); });

        return success.report(__func__);
      }

    private:
      static std::string minimalCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "minimal",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "buses": [
            {"number": 1, "class": "emt_bus", "init": {"Va": 1.0, "Vb": 0.0, "Vc": 0.0}}
          ],
          "branches": [],
          "devices": [],
          "signals": [],
          "extension": {}
        })";
      }

      static std::string duplicateBusCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "duplicate_bus",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "buses": [
            {"number": 1, "class": "emt_bus"},
            {"number": 1, "class": "emt_bus"}
          ],
          "branches": [],
          "devices": [],
          "signals": []
        })";
      }

      static std::string duplicateDeviceCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "duplicate_device",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "buses": [],
          "branches": [],
          "devices": [
            {"id": "load", "class": "ShuntLoad"},
            {"id": "load", "class": "ShuntLoad"}
          ],
          "signals": []
        })";
      }

      static std::string duplicateModelCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "duplicate_model",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "models": {
            "Yc": [
              {"id": "yc"},
              {"id": "yc"}
            ]
          },
          "buses": [],
          "branches": [],
          "devices": [],
          "signals": []
        })";
      }

      static std::string withoutSignalsCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "missing_signals",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "buses": [],
          "branches": [],
          "devices": []
        })";
      }
    };

  } // namespace Testing
} // namespace GridKit
