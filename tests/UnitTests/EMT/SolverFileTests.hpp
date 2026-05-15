#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/IO/SolverFile.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class RealT, typename IdxT>
    class EMTSolverFileTests
    {
    public:
      using Json = nlohmann::json;
      using Data = EMT::CaseData<RealT, IdxT>;

      TestOutcome parserHappyPath()
      {
        TestStatus success = true;

        const auto dir  = std::filesystem::path("EMTSolverFileParserTest");
        const auto file = writeJson(dir, "case.solver.json", solverJson());

        const auto solver  = EMT::IO::readSolverFile(file);
        success           *= (solver.format_version == 1);
        success           *= (solver.case_file == dir / "TwoBus.case.json");
        success           *= isEqual(solver.solve.t0, 0.0);
        success           *= isEqual(solver.solve.tmax, 0.06);
        success           *= isEqual(solver.solve.dt, 1.0e-4);
        success           *= isEqual(solver.solve.rel_tol, 1.0e-8);
        success           *= isEqual(solver.solve.abs_tol, 1.0e-8);
        success           *= (solver.solve.max_steps == 200000u);
        success           *= solver.solve.use_jacobian;
        success           *= solver.output.monitor.has_value();
        if (solver.output.monitor)
        {
          success *= (solver.output.monitor->file == dir / "TwoBus.csv");
        }
        success *= solver.output.ida.has_value();
        if (solver.output.ida)
        {
          success *= (solver.output.ida->file == dir / "TwoBus.ida.json");
          success *= solver.output.ida->log.has_value();
          if (solver.output.ida->log)
          {
            success *= (solver.output.ida->log->file == dir / "TwoBus.ida.log");
            success *= (solver.output.ida->log->level == "warning");
          }
        }
        success *= solver.validation.has_value();
        if (solver.validation)
        {
          success *= (solver.validation->reference_file == dir / "TwoBus.ref.csv");
          success *= isEqual(solver.validation->error_tolerance, 1.0e-4);
        }
        success *= (solver.schedule.size() == 3u);
        if (solver.schedule.size() == 3u)
        {
          success           *= std::holds_alternative<GridKit::Model::Events::Fault>(solver.schedule[0].action);
          success           *= std::holds_alternative<GridKit::Model::Events::Clear>(solver.schedule[1].action);
          success           *= std::holds_alternative<GridKit::Model::Events::Open>(solver.schedule[2].action);
          const auto& fault  = std::get<GridKit::Model::Events::Fault>(solver.schedule[0].action);
          success           *= isEqual(fault.r, 15.0);
          success           *= fault.phases.includes(0);
          success           *= fault.phases.includes(1);
          success           *= fault.phases.includes(2);
        }

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome parserDefaults()
      {
        TestStatus success = true;

        const auto dir  = std::filesystem::path("EMTSolverFileDefaultsTest");
        const auto file = writeJson(dir,
                                    "minimal.solver.json",
                                    Json{{"format_version", 1},
                                         {"case_file", "case.json"},
                                         {"solve", Json{{"tmax", 1.0}, {"dt", 0.01}}}});

        const auto solver  = EMT::IO::readSolverFile(file);
        success           *= isEqual(solver.solve.t0, 0.0);
        success           *= isEqual(solver.solve.rel_tol, 1.0e-8);
        success           *= isEqual(solver.solve.abs_tol, 1.0e-8);
        success           *= (solver.solve.max_steps == 200000u);
        success           *= solver.solve.use_jacobian;
        success           *= !solver.output.monitor.has_value();
        success           *= !solver.output.ida.has_value();
        success           *= solver.schedule.empty();
        success           *= !solver.validation.has_value();

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

      TestOutcome parserErrors()
      {
        TestStatus success = true;

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["bogus"] = 1;
                }),
            "unknown key 'bogus'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json.erase("format_version");
                }),
            "solver file.format_version");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["format_version"] = 2;
                }),
            "unsupported version 2");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["case_file"] = "";
                }),
            "solver file.case_file");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["solve"].erase("tmax");
                }),
            "solve.tmax");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["solve"]["dt"] = 0.0;
                }),
            "solve.dt");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["output"]["ida"]["format"] = "text";
                }),
            "unknown key 'format'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["output"]["ida_stats"] = Json{{"file", "old.json"}};
                }),
            "unknown key 'ida_stats'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["output"]["ida-stats"] = Json{{"file", "old.json"}};
                }),
            "unknown key 'ida-stats'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["output"]["ida_diagnostics"] = Json{{"file", "old.json"}};
                }),
            "unknown key 'ida_diagnostics'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["output"]["ida"]["log"]["level"] = "trace";
                }),
            "output.ida.log.level");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["schedule"][0]["action"] = "energize";
                }),
            "unknown action");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["schedule"][0]["params"]["phases"] = "ad";
                }),
            "only 'a', 'b', and 'c'");

        success *= solverErrorContains(
            withMutation(
                [](Json& json)
                {
                  json["schedule"][0]["time"] = 9.0;
                }),
            "exceeds solve.tmax");

        return success.report(__func__);
      }

      TestOutcome scheduleAndOutput()
      {
        TestStatus success = true;

        auto loaded = EMT::loadCase<Data>(breakerCaseJson());
        auto solver = EMT::IO::readSolverFile(writeJson("EMTSolverFileScheduleTest",
                                                        "case.solver.json",
                                                        breakerSolverJson()));

        EMT::IO::applyOutput(loaded.data, solver.output);
        success *= (loaded.data.monitor_sinks.size() == 1u);
        if (!loaded.data.monitor_sinks.empty())
        {
          success *= (loaded.data.monitor_sinks[0].file_name
                      == std::filesystem::path("EMTSolverFileScheduleTest/out.csv").string());
        }

        EMT::IO::installSchedule(loaded, solver.schedule);
        success *= (loaded.data.events.size() == 4u);

        EMT::SystemModel<Data> system(std::move(loaded.data));
        success *= doesNotThrow(
            [&]()
            {
              system.allocate();
            });

        auto unsupported = EMT::loadCase<Data>(breakerCaseJson());
        auto bad_solver  = solver;
        bad_solver.schedule.clear();
        bad_solver.schedule.push_back({0.1,
                                       "breaker",
                                       GridKit::Model::Events::Fault{GridKit::Model::Events::PhaseMask::abc(), 1.0, 0.0, 0.0},
                                       0u});
        EMT::IO::installSchedule(unsupported, bad_solver.schedule);
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> bad_system(std::move(unsupported.data));
              bad_system.allocate();
            });

        auto ambiguous = EMT::loadCase<Data>(breakerCaseJson());
        ambiguous.data.addMonitorSink({"a.csv", GridKit::Model::VariableMonitorFormat::CSV, ","});
        ambiguous.data.addMonitorSink({"b.csv", GridKit::Model::VariableMonitorFormat::CSV, ","});
        success *= callErrorContains(
            [&]()
            {
              EMT::IO::applyOutput(ambiguous.data, solver.output);
            },
            "multiple monitor sinks");

        std::filesystem::remove_all("EMTSolverFileScheduleTest");
        return success.report(__func__);
      }

    private:
      static Json solverJson()
      {
        return Json::parse(R"json(
{
  "format_version": 1,
  "case_file": "TwoBus.case.json",
  "solve": {
    "tmax": 0.06,
    "dt": 1e-4,
    "rel_tol": 1e-8,
    "abs_tol": 1e-8,
    "max_steps": 200000,
    "use_jacobian": true
  },
  "output": {
    "monitor": { "file": "TwoBus.csv" },
    "ida": { "file": "TwoBus.ida.json", "log": { "file": "TwoBus.ida.log" } }
  },
  "schedule": [
    { "time": 0.010, "target": "receiving_bus", "action": "fault", "params": { "r": 15.0, "phases": "abc" } },
    { "time": 0.011, "target": "receiving_bus", "action": "clear" },
    { "time": 0.011, "target": "breaker", "action": "open" }
  ],
  "validation": {
    "reference_file": "TwoBus.ref.csv",
    "error_tolerance": 1e-4
  }
}
)json");
      }

      static Json breakerSolverJson()
      {
        auto json        = solverJson();
        json["output"]   = Json{{"monitor", Json{{"file", "out.csv"}}}};
        json["schedule"] = Json::array({Json{{"time", 0.010}, {"target", "receiving_bus"}, {"action", "fault"}, {"params", Json{{"r", 15.0}}}},
                                        Json{{"time", 0.011}, {"target", "receiving_bus"}, {"action", "clear"}},
                                        Json{{"time", 0.012}, {"target", "breaker"}, {"action", "open"}},
                                        Json{{"time", 0.013}, {"target", "breaker"}, {"action", "close"}}});
        json.erase("validation");
        return json;
      }

      static Json breakerCaseJson()
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
      "name": "breaker",
      "class": "Breaker",
      "params": {},
      "ports": { "from": "source_bus", "to": "receiving_bus" }
    }
  ]
}
)json");
      }

      template <class Fn>
      static Json withMutation(Fn&& fn)
      {
        auto json = solverJson();
        fn(json);
        return json;
      }

      static std::filesystem::path writeJson(const std::filesystem::path& dir,
                                             const std::string&           name,
                                             const Json&                  json)
      {
        std::filesystem::remove_all(dir);
        std::filesystem::create_directory(dir);
        const auto    file = dir / name;
        std::ofstream output(file);
        output << json.dump(2);
        return file;
      }

      static bool solverErrorContains(const Json& json, std::string_view text)
      {
        const auto dir  = std::filesystem::path("EMTSolverFileErrorTest");
        const auto file = writeJson(dir, "bad.solver.json", json);
        const auto ok   = callErrorContains(
            [&]()
            {
              (void) EMT::IO::readSolverFile(file);
            },
            text);
        std::filesystem::remove_all(dir);
        return ok;
      }

      template <class Fn>
      static bool callErrorContains(Fn&& fn, std::string_view text)
      {
        try
        {
          fn();
        }
        catch (const std::exception& ex)
        {
          return std::string(ex.what()).find(text) != std::string::npos;
        }
        return false;
      }

      template <class Fn>
      static bool doesNotThrow(Fn&& fn)
      {
        try
        {
          fn();
        }
        catch (...)
        {
          return false;
        }
        return true;
      }
    };
  } // namespace Testing
} // namespace GridKit
