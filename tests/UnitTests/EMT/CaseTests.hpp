#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <tuple>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    namespace EMTCaseMocks
    {
      template <class RealT, typename IdxT>
      struct ProducerData
      {
      };

      template <class RealT, typename IdxT>
      struct ConsumerData
      {
      };

      template <class RealT, typename IdxT>
      struct DuplicatePortData
      {
      };

      template <class RealT, typename IdxT>
      struct Producer
      {
        using Data = ProducerData<RealT, IdxT>;

        static constexpr size_t variable_count        = 1;
        static constexpr size_t equation_count        = 1;
        static constexpr size_t electrical_port_count = 0;
        static constexpr size_t input_port_count      = 0;
        static constexpr size_t output_port_count     = 1;

        Producer() = default;

        explicit Producer(Data)
        {
        }

        static constexpr bool differential(size_t)
        {
          return true;
        }

        static constexpr EMT::OutputPortSpec outputPort(size_t)
        {
          return {0};
        }
      };

      template <class RealT, typename IdxT>
      struct Consumer
      {
        using Data = ConsumerData<RealT, IdxT>;

        static constexpr size_t variable_count        = 1;
        static constexpr size_t equation_count        = 1;
        static constexpr size_t electrical_port_count = 0;
        static constexpr size_t input_port_count      = 1;
        static constexpr size_t output_port_count     = 0;

        Consumer() = default;

        explicit Consumer(Data)
        {
        }

        static constexpr bool differential(size_t)
        {
          return true;
        }
      };

      template <class RealT, typename IdxT>
      struct DuplicatePorts
      {
        using Data = DuplicatePortData<RealT, IdxT>;

        static constexpr size_t variable_count        = 0;
        static constexpr size_t equation_count        = 0;
        static constexpr size_t electrical_port_count = 1;
        static constexpr size_t input_port_count      = 0;
        static constexpr size_t output_port_count     = 1;

        DuplicatePorts() = default;

        explicit DuplicatePorts(Data)
        {
        }

        static constexpr EMT::OutputPortSpec outputPort(size_t)
        {
          return {0};
        }
      };
    } // namespace EMTCaseMocks
  } // namespace Testing
} // namespace GridKit

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct ComponentDescriptor<Testing::EMTCaseMocks::Producer<RealT, IdxT>>
    {
      using Component = Testing::EMTCaseMocks::Producer<RealT, IdxT>;
      using Data      = Testing::EMTCaseMocks::ProducerData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "MockPortProducer";
      static constexpr std::array<std::string_view, 0> electrical_ports{};
      static constexpr std::array<std::string_view, 0> input_ports{};
      static constexpr std::array<std::string_view, 1> output_ports{"out"};
      static constexpr auto                            params = std::tuple{};

      static_assert(electrical_ports.size() == ComponentTraits<Component>::electrical_port_count);
      static_assert(input_ports.size() == ComponentTraits<Component>::input_port_count);
      static_assert(output_ports.size() == ComponentTraits<Component>::output_port_count);
    };

    template <class RealT, typename IdxT>
    struct ComponentDescriptor<Testing::EMTCaseMocks::Consumer<RealT, IdxT>>
    {
      using Component = Testing::EMTCaseMocks::Consumer<RealT, IdxT>;
      using Data      = Testing::EMTCaseMocks::ConsumerData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "MockPortConsumer";
      static constexpr std::array<std::string_view, 0> electrical_ports{};
      static constexpr std::array<std::string_view, 1> input_ports{"in"};
      static constexpr std::array<std::string_view, 0> output_ports{};
      static constexpr auto                            params = std::tuple{};

      static_assert(electrical_ports.size() == ComponentTraits<Component>::electrical_port_count);
      static_assert(input_ports.size() == ComponentTraits<Component>::input_port_count);
      static_assert(output_ports.size() == ComponentTraits<Component>::output_port_count);
    };

    template <class RealT, typename IdxT>
    struct ComponentDescriptor<Testing::EMTCaseMocks::DuplicatePorts<RealT, IdxT>>
    {
      using Component = Testing::EMTCaseMocks::DuplicatePorts<RealT, IdxT>;
      using Data      = Testing::EMTCaseMocks::DuplicatePortData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "MockDuplicatePorts";
      static constexpr std::array<std::string_view, 1> electrical_ports{"same"};
      static constexpr std::array<std::string_view, 0> input_ports{};
      static constexpr std::array<std::string_view, 1> output_ports{"same"};
      static constexpr auto                            params = std::tuple{};

      static_assert(electrical_ports.size() == ComponentTraits<Component>::electrical_port_count);
      static_assert(input_ports.size() == ComponentTraits<Component>::input_port_count);
      static_assert(output_ports.size() == ComponentTraits<Component>::output_port_count);
    };
  } // namespace EMT
} // namespace GridKit

namespace GridKit
{
  namespace Testing
  {
    template <class RealT, typename IdxT>
    class EMTCaseTests
    {
    public:
      using Json = nlohmann::json;
      using Data = EMT::CaseData<RealT, IdxT>;

      TestOutcome happyPath()
      {
        TestStatus success = true;

        auto loaded = EMT::loadCase<Data>(twoBusJson());

        success *= (loaded.names.buses.size() == 2u);
        success *= (loaded.names.components.size() == 3u);
        success *= (loaded.names.bus("source_bus") == IdxT{0});
        success *= (loaded.names.bus("receiving_bus") == IdxT{1});
        success *= (loaded.data.components.template get<EMT::VoltageSource<RealT, IdxT>>().size() == 1u);
        success *= (loaded.data.components.template get<EMT::LoadRL<RealT, IdxT>>().size() == 1u);
        success *= (loaded.data.components.template get<EMT::BranchLumpedConstant<RealT, IdxT>>().size() == 1u);
        success *= (loaded.data.port_connections.size() == 4u);
        success *= loaded.data.signal_port_connections.empty();
        success *= (loaded.data.monitor_sinks.size() == 1u);
        success *= (loaded.data.bus_monitors.size() == 2u);
        success *= (loaded.data.component_monitors.size() == 3u);
        success *= (loaded.data.bus_monitors[1].label == "receiving_bus");
        success *= (loaded.data.bus_monitors[1].variables.size() == 6u);

        EMT::SystemModel<Data> system(std::move(loaded.data), RealT{1.0e-8}, RealT{1.0e-8});
        system.allocate();
        system.initialize();
        system.updateTime(0.0, 1.0);
        system.evaluateResidual();
        system.evaluateJacobian();

        success *= (system.size() == 12);
        success *= (system.nnz() == 30);
        success *= (maxAbs(system.getResidual()) < RealT{1.0e-8});

        return success.report(__func__);
      }

      TestOutcome signalPortWiring()
      {
        TestStatus success = true;

        using Producer = EMTCaseMocks::Producer<RealT, IdxT>;
        using Consumer = EMTCaseMocks::Consumer<RealT, IdxT>;
        using Data     = EMT::SystemModelData<RealT, IdxT, Producer, Consumer>;

        auto loaded = EMT::loadCase<Data>(portCaseJson());

        success *= (loaded.data.signal_port_connections.size() == 1u);
        if (!loaded.data.signal_port_connections.empty())
        {
          const auto  producer  = loaded.names.component("producer");
          const auto  consumer  = loaded.names.component("consumer");
          const auto& conn      = loaded.data.signal_port_connections.front();
          success              *= (conn.output.component == producer.id);
          success              *= (conn.output.index == 0u);
          success              *= (conn.input.component == consumer.id);
          success              *= (conn.input.index == 0u);
        }

        success *= caseErrorContains(
            [&]()
            {
              auto bad                            = portCaseJson();
              bad["components"][1]["ports"]["in"] = "missing.out";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown component 'missing'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                            = portCaseJson();
              bad["components"][1]["ports"]["in"] = "producer.bad";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown output port 'producer.bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                      = portCaseJson();
              bad["components"][1]["ports"] = Json{{"bad", "producer.out"}};
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown port 'bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                            = portCaseJson();
              bad["components"][1]["ports"]["in"] = "producer";
              (void) EMT::loadCase<Data>(bad);
            },
            "expected port reference");

        using Duplicate      = EMTCaseMocks::DuplicatePorts<RealT, IdxT>;
        using DuplicateData  = EMT::SystemModelData<RealT, IdxT, Duplicate>;
        success             *= caseErrorContains(
            [&]()
            {
              (void) EMT::loadCase<DuplicateData>(duplicatePortCaseJson());
            },
            "duplicate port 'same'");

        return success.report(__func__);
      }

      TestOutcome pathLoading()
      {
        TestStatus success = true;

        const auto dir  = std::filesystem::path("EMTCasePathTest");
        const auto file = dir / "case.case.json";
        std::filesystem::remove_all(dir);
        std::filesystem::create_directory(dir);

        {
          std::ofstream out(file);
          out << twoBusJson().dump(2);
        }

        auto loaded  = EMT::loadCase<Data>(file);
        success     *= (loaded.data.monitor_sinks.size() == 1u);
        if (!loaded.data.monitor_sinks.empty())
        {
          success *= (loaded.data.monitor_sinks[0].file_name == (dir / "EMTTinyTwoBus.csv").string());
        }

        const auto bad_file = dir / "bad.case.json";
        {
          std::ofstream out(bad_file);
          out << "{ bad json";
        }
        success *= caseErrorContains(
            [&]()
            {
              (void) EMT::loadCase<Data>(bad_file);
            },
            "malformed JSON");

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome frequencyDependentBranchCase()
      {
        TestStatus success = true;

        const auto dir      = std::filesystem::path("EMTFDCaseTest");
        const auto fit_dir  = dir / "fits";
        const auto fit_file = fit_dir / "unit.yc.fit.json";
        const auto case_file = dir / "fd.case.json";
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(fit_dir);

        {
          std::ofstream out(fit_file);
          out << constantYcFitJson().dump(2) << '\n';
        }
        {
          std::ofstream out(case_file);
          out << frequencyDependentCaseJson().dump(2) << '\n';
        }

        auto loaded = EMT::loadCase<Data>(case_file);
        success *= (loaded.data.components.template get<EMT::BranchFrequencyDependent<RealT, IdxT>>().size() == 1u);
        success *= (loaded.data.port_connections.size() == 2u);
        success *= (loaded.data.component_monitors.size() == 1u);

        EMT::SystemModel<Data> system(std::move(loaded.data), RealT{1.0e-8}, RealT{1.0e-8});
        system.allocate();
        system.initialize();
        system.updateTime(0.0, 1.0);
        system.evaluateResidual();
        system.evaluateJacobian();

        success *= caseErrorContains(
            [&]()
            {
              auto bad = frequencyDependentCaseJson();
              bad["components"][0]["params"]["propagation"] = Json::object();
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown key 'propagation'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = frequencyDependentCaseJson();
              bad["components"][0]["params"]["yc"]["extra"] = 1;
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown key 'extra'");

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome errorCases()
      {
        TestStatus success = true;

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad.erase("header");
              (void) EMT::loadCase<Data>(bad);
            },
            "header");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                        = twoBusJson();
              bad["header"]["format_version"] = 2;
              (void) EMT::loadCase<Data>(bad);
            },
            "unsupported version 2");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                     = twoBusJson();
              bad["components"][0]["name"] = "receiving_bus";
              (void) EMT::loadCase<Data>(bad);
            },
            "already used");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                = twoBusJson();
              bad["buses"][0]["name"] = "bad-name";
              (void) EMT::loadCase<Data>(bad);
            },
            "identifier");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["buses"][0].erase("init");
              (void) EMT::loadCase<Data>(bad);
            },
            "bus[0].init");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["buses"][0]["init"].erase("vm");
              (void) EMT::loadCase<Data>(bad);
            },
            "bus[0].init.vm");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["buses"][0]["init"].erase("va");
              (void) EMT::loadCase<Data>(bad);
            },
            "bus[0].init.va");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                      = twoBusJson();
              bad["components"][0]["class"] = "NoSuchComponent";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown class");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                                = twoBusJson();
              bad["components"][1]["params"]["bogus"] = 1.0;
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown key 'bogus'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["components"][1]["params"].erase("length");
              (void) EMT::loadCase<Data>(bad);
            },
            "length");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                            = twoBusJson();
              bad["components"][2]["params"]["r"] = Json::array({-1.0, 25.0, 25.0});
              (void) EMT::loadCase<Data>(bad);
            },
            "resistance");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                            = twoBusJson();
              bad["components"][1]["ports"]["to"] = "missing_bus";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown bus 'missing_bus'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                             = twoBusJson();
              bad["components"][1]["ports"]["bad"] = "receiving_bus";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown port 'bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["components"][1]["ports"].erase("to");
              (void) EMT::loadCase<Data>(bad);
            },
            "missing port 'to'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                  = twoBusJson();
              bad["buses"][0]["mon"][0] = "vx";
              (void) EMT::loadCase<Data>(bad);
            },
            "monitor variable 'vx'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                       = twoBusJson();
              bad["components"][1]["mon"][0] = "vx";
              (void) EMT::loadCase<Data>(bad);
            },
            "monitor variable 'vx'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                     = twoBusJson();
              bad["monitors"][0]["format"] = "binary";
              (void) EMT::loadCase<Data>(bad);
            },
            "unsupported monitor format");

        return success.report(__func__);
      }

    private:
      static Json twoBusJson()
      {
        return Json::parse(R"json(
{
  "header": {
    "format_version": 1,
    "case_name": "EMT Tiny TwoBus",
    "description": "Two-bus EMT test case"
  },
  "buses": [
    { "name": "source_bus", "init": { "vm": 120.32988891054056, "va": 0.0095877412086657603 },
      "mon": ["VA", "VB", "VC"] },
    { "name": "receiving_bus", "init": { "vm": 120.0, "va": 0.0 },
      "mon": ["va", "vb", "vc", "ifa", "ifb", "ifc"] }
  ],
  "components": [
    {
      "name": "source",
      "class": "VoltageSource",
      "params": {
        "e": [120.63846086344611, 120.63846086344611, 120.63846086344611],
        "phi": [0.011405793511148577, -2.0829893088820466, 2.1058008959043439],
        "r": [0.10, 0.10, 0.10],
        "frequency": 60.0
      },
      "ports": { "bus": "source_bus" },
      "mon": ["IA", "IB", "IC"]
    },
    {
      "name": "line",
      "class": "BranchLumpedConstant",
      "params": {
        "r": [[0.10, 0.0, 0.0], [0.0, 0.10, 0.0], [0.0, 0.0, 0.10]],
        "l": [[1.0e-3, 0.0, 0.0], [0.0, 1.0e-3, 0.0], [0.0, 0.0, 1.0e-3]],
        "g": [[2.0e-4, 0.0, 0.0], [0.0, 2.0e-4, 0.0], [0.0, 0.0, 2.0e-4]],
        "c": [[1.0e-4, 0.0, 0.0], [0.0, 1.0e-4, 0.0], [0.0, 0.0, 1.0e-4]],
        "length": 1.0
      },
      "ports": { "from": "source_bus", "to": "receiving_bus" },
      "mon": ["ia", "ib", "ic"]
    },
    {
      "name": "load",
      "class": "LoadRL",
      "params": {
        "r": [25.0, 25.0, 25.0],
        "l": [5.0e-2, 5.0e-2, 5.0e-2]
      },
      "ports": { "ac": "receiving_bus" },
      "mon": ["ia", "ib", "ic"]
    }
  ],
  "monitors": [{ "file_name": "EMTTinyTwoBus.csv", "format": "csv" }]
}
)json");
      }

      static Json portCaseJson()
      {
        return Json::parse(R"json(
{
  "header": { "format_version": 1 },
  "buses": [],
  "components": [
    { "name": "producer", "class": "MockPortProducer", "params": {}, "ports": {} },
    { "name": "consumer", "class": "MockPortConsumer", "params": {},
      "ports": { "in": "producer.out" } }
  ]
}
)json");
      }

      static Json duplicatePortCaseJson()
      {
        return Json::parse(R"json(
{
  "header": { "format_version": 1 },
  "buses": [{ "name": "bus", "init": { "vm": 1.0, "va": 0.0 } }],
  "components": [
    { "name": "duplicate", "class": "MockDuplicatePorts", "params": {}, "ports": { "same": "bus" } }
  ]
}
)json");
      }

      static Json frequencyDependentCaseJson()
      {
        return Json::parse(R"json(
{
  "header": { "format_version": 1 },
  "buses": [
    { "name": "source_bus", "init": { "vm": 120.0, "va": 0.0 } },
    { "name": "receiving_bus", "init": { "vm": 120.0, "va": 0.0 } }
  ],
  "components": [
    {
      "name": "line",
      "class": "BranchFrequencyDependent",
      "params": {
        "length": 1000.0,
        "phase_order": ["a", "b", "c"],
        "yc": {
          "fit_file": "fits/unit.yc.fit.json"
        }
      },
      "ports": { "from": "source_bus", "to": "receiving_bus" },
      "mon": ["ifa", "ifb", "ifc", "ita", "itb", "itc"]
    }
  ]
}
)json");
      }

      static Json constantYcFitJson()
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
                  {"d", Json::array({0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001})},
                  {"e", Json::array({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
                  {"real_poles", Json::array()},
                  {"real_residues", Json::array()},
                  {"pair_real", Json::array()},
                  {"pair_imag", Json::array()},
                  {"pair_residue_real", Json::array()},
                  {"pair_residue_imag", Json::array()}}},
            {"validation", Json::object()},
            {"metadata", Json::object()}};
      }

      template <class Vector>
      static RealT maxAbs(const Vector& values)
      {
        RealT norm = 0.0;
        for (const auto& value : values)
        {
          norm = std::max(norm, std::abs(value));
        }
        return norm;
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
    };
  } // namespace Testing
} // namespace GridKit
