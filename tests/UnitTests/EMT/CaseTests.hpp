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
      struct Producer
      {
        using Data = ProducerData<RealT, IdxT>;

        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 0;
        static constexpr size_t output_count   = 1;

        Producer() = default;

        explicit Producer(Data)
        {
        }

        static constexpr bool differential(size_t)
        {
          return true;
        }

        static constexpr EMT::SignalOutputSpec output(size_t)
        {
          return {0};
        }
      };

      template <class RealT, typename IdxT>
      struct Consumer
      {
        using Data = ConsumerData<RealT, IdxT>;

        static constexpr size_t variable_count = 1;
        static constexpr size_t equation_count = 1;
        static constexpr size_t terminal_count = 0;
        static constexpr size_t input_count    = 1;
        static constexpr size_t output_count   = 0;

        Consumer() = default;

        explicit Consumer(Data)
        {
        }

        static constexpr bool differential(size_t)
        {
          return true;
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
      static constexpr std::array<std::string_view, 0> terminals{};
      static constexpr std::array<std::string_view, 0> inputs{};
      static constexpr std::array<std::string_view, 1> outputs{"out"};
      static constexpr auto                            params = std::tuple{};

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
    };

    template <class RealT, typename IdxT>
    struct ComponentDescriptor<Testing::EMTCaseMocks::Consumer<RealT, IdxT>>
    {
      using Component = Testing::EMTCaseMocks::Consumer<RealT, IdxT>;
      using Data      = Testing::EMTCaseMocks::ConsumerData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "MockPortConsumer";
      static constexpr std::array<std::string_view, 0> terminals{};
      static constexpr std::array<std::string_view, 1> inputs{"in"};
      static constexpr std::array<std::string_view, 0> outputs{};
      static constexpr auto                            params = std::tuple{};

      static_assert(terminals.size() == ComponentTraits<Component>::terminal_count);
      static_assert(inputs.size() == ComponentTraits<Component>::input_count);
      static_assert(outputs.size() == ComponentTraits<Component>::output_count);
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
        success *= (loaded.data.terminal_connections.size() == 4u);
        success *= loaded.data.signal_connections.empty();
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

        success *= (loaded.data.signal_connections.size() == 1u);
        if (!loaded.data.signal_connections.empty())
        {
          const auto  producer  = loaded.names.component("producer");
          const auto  consumer  = loaded.names.component("consumer");
          const auto& conn      = loaded.data.signal_connections.front();
          success              *= (conn.output.component == producer.id);
          success              *= (conn.output.index == 0u);
          success              *= (conn.input.component == consumer.id);
          success              *= (conn.input.index == 0u);
        }

        success *= caseErrorContains(
            [&]()
            {
              auto bad                             = portCaseJson();
              bad["components"][1]["inputs"]["in"] = "missing.out";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown component 'missing'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                             = portCaseJson();
              bad["components"][1]["inputs"]["in"] = "producer.bad";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown output 'producer.bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                       = portCaseJson();
              bad["components"][1]["inputs"] = Json{{"bad", "producer.out"}};
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown input 'bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                             = portCaseJson();
              bad["components"][1]["inputs"]["in"] = "producer";
              (void) EMT::loadCase<Data>(bad);
            },
            "expected input reference");

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
              auto bad                                = twoBusJson();
              bad["components"][1]["terminals"]["to"] = "missing_bus";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown bus 'missing_bus'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad                                 = twoBusJson();
              bad["components"][1]["terminals"]["bad"] = "receiving_bus";
              (void) EMT::loadCase<Data>(bad);
            },
            "unknown terminal 'bad'");

        success *= caseErrorContains(
            [&]()
            {
              auto bad = twoBusJson();
              bad["components"][1]["terminals"].erase("to");
              (void) EMT::loadCase<Data>(bad);
            },
            "missing terminal 'to'");

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
    "description": "Two-bus EMT test case",
    "frequency": 60.0
  },
  "buses": [
    { "name": "source_bus", "vm0": 120.32988891054056, "va0": 0.0095877412086657603,
      "mon": ["VA", "VB", "VC"] },
    { "name": "receiving_bus", "vm0": 120.0,           "va0": 0.0,
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
      "terminals": { "ac": "source_bus" },
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
      "terminals": { "from": "source_bus", "to": "receiving_bus" },
      "mon": ["ia", "ib", "ic"]
    },
    {
      "name": "load",
      "class": "LoadRL",
      "params": {
        "r": [25.0, 25.0, 25.0],
        "l": [5.0e-2, 5.0e-2, 5.0e-2]
      },
      "terminals": { "ac": "receiving_bus" },
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
    { "name": "producer", "class": "MockPortProducer", "params": {}, "terminals": {} },
    { "name": "consumer", "class": "MockPortConsumer", "params": {}, "terminals": {},
      "inputs": { "in": "producer.out" } }
  ]
}
)json");
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
