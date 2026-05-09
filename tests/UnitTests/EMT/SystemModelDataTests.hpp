#pragma once

#include <stdexcept>
#include <string>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    class SystemModelDataTests
    {
    public:
      TestOutcome parsesEmtCase()
      {
        TestStatus success = true;

        const auto data = EMT::parseSystemModelDataFromString(validCase());
        const auto tol  = 1.0e-12;

        success *= (data.case_name == "emt_parser");
        success *= (data.bus.size() == 2);
        success *= (data.line.size() == 1);
        success *= (data.shunt_load.size() == 1);
        success *= (data.voltage_source.size() == 1);
        success *= (data.line[0].id == "L12");
        success *= (data.line[0].bus1 == 1);
        success *= (data.line[0].bus2 == 2);
        success *= isEqual(data.line[0].data.characteristic_admittance.p[0], -900.0, tol);
        success *= isEqual(data.line[0].data.characteristic_admittance.complex_p_imag[0],
                           1200.0,
                           tol);
        success *= (data.shunt_load[0].id == "load_extra");
        success *= !data.shunt_load[0].data.closed;
        success *= isEqual(data.shunt_load[0].data.conductance[0], 2.0, tol);
        success *= isEqual(data.voltage_source[0].data.current0[1], 1.0, tol);

        return success.report(__func__);
      }

      TestOutcome buildsSystemModelFromData()
      {
        TestStatus success = true;

        const auto                       data = EMT::parseSystemModelDataFromString(validCase());
        EMT::SystemModel<double, size_t> system(data);

        auto* load    = system.getShuntLoad("load_extra");
        auto* source  = system.getVoltageSource("source");
        success      *= (load != nullptr);
        success      *= (source != nullptr);
        success      *= !load->status();

        system.cue("load_extra", EMT::Action::On);
        success *= load->status();
        system.cue("load_extra", EMT::Action::Off);
        success *= !load->status();
        success *= throws<std::runtime_error>(
            [&]()
            { system.cue("missing", EMT::Action::On); });
        success *= (system.allocate() == 0);

        return success.report(__func__);
      }

      TestOutcome rejectsInvalidAdmittance()
      {
        TestStatus success = true;

        success *= throws<std::runtime_error>(
            [&]()
            { EMT::parseSystemModelDataFromString(replace(validCase(), "\"dimension\": 3", "\"dimension\": 2")); });
        success *= throws<std::runtime_error>(
            [&]()
            { EMT::parseSystemModelDataFromString(replace(validCase(), "\"pole\": -900.0", "\"pole\": 900.0")); });
        success *= throws<std::runtime_error>(
            [&]()
            { EMT::parseSystemModelDataFromString(replace(validCase(), "\"imag\": 1200.0", "\"imag\": -1200.0")); });
        success *= throws<std::runtime_error>(
            [&]()
            { EMT::parseSystemModelDataFromString(replace(validCase(), "\"c\": [0.2, 0.0, 0.0]", "\"c\": [0.2, 0.0]")); });

        return success.report(__func__);
      }

      TestOutcome rejectsUnresolvedReferences()
      {
        TestStatus success = true;

        success *= throws<std::runtime_error>(
            [&]()
            {
              EMT::parseSystemModelDataFromString(
                  replace(validCase(), "\"Yc\": \"yc\"", "\"Yc\": \"missing\""));
            });
        success *= throws<std::runtime_error>(
            [&]()
            {
              EMT::parseSystemModelDataFromString(
                  replace(validCase(), "\"bus2\": 2", "\"bus2\": 3"));
            });

        return success.report(__func__);
      }

    private:
      static std::string replace(std::string        input,
                                 const std::string& source,
                                 const std::string& target)
      {
        const auto pos = input.find(source);
        if (pos == std::string::npos)
        {
          throw std::logic_error("test input token not found");
        }
        input.replace(pos, source.size(), target);
        return input;
      }

      static std::string validCase()
      {
        return R"({
          "header": {
            "format_version": 1,
            "format_revision": 0,
            "case_name": "emt_parser",
            "case_description": "",
            "case_comments": "",
            "freq_base": 60.0,
            "va_base": 100000000.0
          },
          "models": {
            "Yc": [
              {
                "id": "yc",
                "dimension": 3,
                "d": [
                  [1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]
                ],
                "e": [
                  [0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0]
                ],
                "real_poles": [
                  {"pole": -900.0, "b": [1.0, 0.0, 0.0], "c": [0.2, 0.0, 0.0]}
                ],
                "complex_pole_pairs": [
                  {
                    "pole": {"real": -250.0, "imag": 1200.0},
                    "b": {"real": [0.0, 1.0, 0.0], "imag": [0.0, 0.0, 0.0]},
                    "c": {"real": [0.0, 0.3, 0.0], "imag": [0.0, 0.0, 0.0]}
                  }
                ]
              }
            ]
          },
          "buses": [
            {"number": 1, "class": "emt_bus", "init": {"Va": 1.0, "Vb": 0.0, "Vc": 0.0}},
            {"number": 2, "class": "emt_bus", "init": {"Va": 0.9, "Vb": 0.0, "Vc": 0.0}}
          ],
          "branches": [
            {
              "id": "L12",
              "class": "Line",
              "ports": {"bus1": 1, "bus2": 2},
              "params": {"Yc": "yc"}
            }
          ],
          "devices": [
            {
              "id": "source",
              "class": "VoltageSource",
              "ports": {"bus": 1},
              "params": {"amplitude": 1.0, "frequency": 60.0, "phase": 0.0},
              "init": {"current": [0.0, 1.0, -1.0]}
            },
            {
              "id": "load_extra",
              "class": "ShuntLoad",
              "ports": {"bus": 2},
              "params": {
                "conductance": [
                  [2.0, 0.0, 0.0],
                  [0.0, 2.0, 0.0],
                  [0.0, 0.0, 2.0]
                ]
              },
              "init": {"closed": false}
            }
          ],
          "signals": []
        })";
      }
    };

  } // namespace Testing
} // namespace GridKit
