#pragma once

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelDataJSONParser.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTSystemModelTests
    {
    public:
      using ModelT     = EMT::SystemModel<ScalarT, IdxT>;
      using ModelDataT = typename ModelT::ModelDataT;
      using RealT      = typename ModelT::RealT;
      using BusT       = typename ModelT::BusT;
      using BusDataT   = typename ModelDataT::BusDataT;

      TestOutcome parser()
      {
        TestStatus success = true;

        const auto data = parseCase();

        success *= data.format_version == 0;
        success *= data.format_revision == 1;
        success *= data.case_name == "emt-source-load-line";
        success *= data.case_description == "EMT parser and system construction smoke case";
        success *= data.case_comments.empty();

        success *= data.bus.size() == 2;
        success *= data.voltage_source.size() == 1;
        success *= data.load_rl.size() == 1;
        success *= data.line_lumped.size() == 1;

        success *= data.bus[0].bus_id == 1;
        success *= data.bus[0].device_class == "bus";
        success *= data.bus[0].name == "bus1";
        success *= isEqual(data.bus[0].v0[0], RealT{10.0});
        success *= isEqual(data.bus[0].v0[1], RealT{20.0});
        success *= isEqual(data.bus[0].v0[2], RealT{30.0});

        success *= data.voltage_source[0].ports.at(EMT::VoltageSourcePorts::bus) == 1;
        success *= data.load_rl[0].ports.at(EMT::LoadRLPorts::bus) == 1;
        success *= data.line_lumped[0].ports.at(EMT::LineLumpedPorts::bus1) == 1;
        success *= data.line_lumped[0].ports.at(EMT::LineLumpedPorts::bus2) == 2;

        return success.report(__func__);
      }

      TestOutcome construction()
      {
        TestStatus success = true;

        auto   data = parseCase();
        ModelT system(data);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.size() == 21;

        auto* bus1  = static_cast<BusT*>(system.getBus(1));
        success    *= bus1 != nullptr;

        success *= isEqual(bus1->I(0), ScalarT{-1.0});
        success *= isEqual(bus1->I(1), ScalarT{-4.0});
        success *= isEqual(bus1->I(2), ScalarT{-9.0});

        return success.report(__func__);
      }

      TestOutcome connectivityErrors()
      {
        TestStatus success = true;

        ModelDataT duplicate_bus;
        duplicate_bus.bus.push_back(makeBusData(1, RealT{1.0}, RealT{2.0}, RealT{3.0}));
        duplicate_bus.bus.push_back(makeBusData(1, RealT{4.0}, RealT{5.0}, RealT{6.0}));

        success *= throws<std::invalid_argument>(
            [&duplicate_bus]()
            { ModelT system(duplicate_bus); });

        auto missing_bus                                                  = parseCase();
        missing_bus.voltage_source[0].ports[EMT::VoltageSourcePorts::bus] = 99;

        success *= throws<std::invalid_argument>(
            [&missing_bus]()
            { ModelT system(missing_bus); });

        return success.report(__func__);
      }

    private:
      static ModelDataT parseCase()
      {
        auto stream = std::istringstream(caseJSON());
        return EMT::parseSystemModelData(stream);
      }

      static BusDataT makeBusData(IdxT bus_id, RealT va, RealT vb, RealT vc)
      {
        BusDataT data;
        data.bus_id = bus_id;
        data.v0     = {va, vb, vc};
        return data;
      }

      static std::string caseJSON()
      {
        return R"({
          "header": {
            "format_version": 0,
            "format_revision": 1,
            "case_name": "emt-source-load-line",
            "case_description": "EMT parser and system construction smoke case",
            "case_comments": ""
          },
          "buses": [
            {
              "number": 1,
              "class": "bus",
              "name": "bus1",
              "init": { "v0": [10.0, 20.0, 30.0] }
            },
            {
              "number": 2,
              "class": "bus",
              "name": "bus2",
              "init": { "v0": [0.0, 0.0, 0.0] }
            }
          ],
          "devices": [
            {
              "class": "VoltageSource",
              "ports": { "bus": 1 },
              "id": "src1",
              "params": {
                "E": [0.0, 0.0, 0.0],
                "phi": [0.0, 0.0, 0.0],
                "omega": 377.0,
                "G": [0.1, 0.2, 0.3]
              }
            },
            {
              "class": "LoadRL",
              "ports": { "bus": 1 },
              "id": "load1",
              "params": {
                "Ra": 1.0,
                "Rb": 1.0,
                "Rc": 1.0,
                "La": 0.01,
                "Lb": 0.01,
                "Lc": 0.01,
                "Iinj": 0.0,
                "theta": 0.0
              }
            },
            {
              "class": "LineLumped",
              "ports": { "bus1": 1, "bus2": 2 },
              "id": "line1",
              "params": {
                "dx": 1.0,
                "i0": [0.0, 0.0, 0.0],
                "Rp": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                "Lp": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                "Gp": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                "Cp": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
              }
            }
          ]
        })";
      }
    };
  } // namespace Testing
} // namespace GridKit
