#pragma once

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/Events.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace IO
    {
      struct Solve
      {
        double      t0{0.0};
        double      tmax{};
        double      dt{};
        double      rel_tol{1.0e-8};
        double      abs_tol{1.0e-8};
        std::size_t max_steps{200000};
        bool        use_jacobian{true};
      };

      struct Event
      {
        double                         time{};
        std::string                    target;
        GridKit::Model::Events::Action action;
        std::size_t                    order{};
      };

      struct Output
      {
        std::optional<std::filesystem::path> monitor;
        std::optional<std::filesystem::path> ida_stats;
      };

      struct Validation
      {
        std::filesystem::path reference_file;
        double                error_tolerance{1.0e-4};
      };

      struct SolverFile
      {
        int                       format_version{1};
        std::filesystem::path     path;
        std::filesystem::path     case_file;
        Solve                     solve;
        Output                    output;
        std::vector<Event>        schedule;
        std::optional<Validation> validation;
      };

      namespace Detail
      {
        using GridKit::EMT::Detail::fieldContext;
        using GridKit::EMT::Detail::readJsonValue;
        using GridKit::EMT::Detail::throwCase;

        inline std::filesystem::path resolvePath(const std::filesystem::path& base,
                                                 std::filesystem::path        path)
        {
          if (!path.empty() && !path.is_absolute() && !base.empty())
          {
            path = base / path;
          }
          return path;
        }

        inline std::filesystem::path readPath(const nlohmann::json&        obj,
                                              std::string_view             key,
                                              std::string_view             entity,
                                              const std::filesystem::path& base)
        {
          const auto raw = require<std::string>(obj, key, entity);
          if (raw.empty())
          {
            Detail::throwCase(Detail::fieldContext(entity, key), "must not be empty");
          }
          return Detail::resolvePath(base, raw);
        }

        inline const nlohmann::json& requireObject(const nlohmann::json& obj,
                                                   std::string_view      key,
                                                   std::string_view      entity)
        {
          if (!obj.is_object())
          {
            Detail::throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            Detail::throwCase(Detail::fieldContext(entity, key), "required field missing");
          }
          if (!it->is_object())
          {
            Detail::throwCase(Detail::fieldContext(entity, key), "expected object");
          }
          return *it;
        }

        inline std::optional<std::reference_wrapper<const nlohmann::json>> optionalObject(
            const nlohmann::json& obj,
            std::string_view      key,
            std::string_view      entity)
        {
          if (!obj.is_object())
          {
            Detail::throwCase(entity, "expected object");
          }
          const auto it = obj.find(std::string(key));
          if (it == obj.end())
          {
            return std::nullopt;
          }
          if (!it->is_object())
          {
            Detail::throwCase(Detail::fieldContext(entity, key), "expected object");
          }
          return std::cref(*it);
        }

        inline void requireFinitePositive(double value, std::string_view entity)
        {
          if (!std::isfinite(value) || value <= 0.0)
          {
            Detail::throwCase(entity, "must be positive and finite");
          }
        }

        inline void requireFiniteNonnegative(double value, std::string_view entity)
        {
          if (!std::isfinite(value) || value < 0.0)
          {
            Detail::throwCase(entity, "must be nonnegative and finite");
          }
        }

        inline std::size_t readMaxSteps(const nlohmann::json& obj,
                                        std::string_view      entity)
        {
          const auto raw = require<long long>(obj, "max_steps", entity);
          if (raw <= 0)
          {
            Detail::throwCase(Detail::fieldContext(entity, "max_steps"), "must be positive");
          }
          return static_cast<std::size_t>(raw);
        }

        template <class T>
        T optionalValue(const nlohmann::json& obj,
                        std::string_view      key,
                        T                     fallback,
                        std::string_view      entity)
        {
          if (obj.find(std::string(key)) == obj.end())
          {
            return fallback;
          }
          return require<T>(obj, key, entity);
        }

        inline GridKit::Model::Events::PhaseMask readPhases(const nlohmann::json& params)
        {
          using PhaseMask = GridKit::Model::Events::PhaseMask;
          const auto it   = params.find("phases");
          if (it == params.end())
          {
            return PhaseMask::abc();
          }
          return Detail::readJsonValue<PhaseMask>(*it, "params.phases");
        }

        inline nlohmann::json emptyObject()
        {
          return nlohmann::json::object();
        }
      } // namespace Detail

      inline GridKit::Model::Events::Action readAction(std::string_view      name,
                                                       const nlohmann::json& params)
      {
        namespace Events = GridKit::Model::Events;

        if (!params.is_object())
        {
          Detail::throwCase("params", "expected object");
        }

        if (name == "open")
        {
          rejectUnknownKeys(params, {"phases"}, "params");
          return Events::Open{Detail::readPhases(params)};
        }
        if (name == "close")
        {
          rejectUnknownKeys(params, {"phases"}, "params");
          return Events::Close{Detail::readPhases(params)};
        }
        if (name == "clear")
        {
          rejectUnknownKeys(params, {"phases"}, "params");
          return Events::Clear{Detail::readPhases(params)};
        }
        if (name == "fault")
        {
          rejectUnknownKeys(params, {"r", "x", "percent", "phases"}, "params");
          const double r = require<double>(params, "r", "params");
          Detail::requireFiniteNonnegative(r, "params.r");
          const double x       = params.contains("x") ? require<double>(params, "x", "params") : 0.0;
          const double percent = params.contains("percent") ? require<double>(params, "percent", "params") : 0.0;
          if (!std::isfinite(x))
          {
            Detail::throwCase("params.x", "must be finite");
          }
          if (!std::isfinite(percent))
          {
            Detail::throwCase("params.percent", "must be finite");
          }
          return Events::Fault{Detail::readPhases(params), r, x, percent};
        }

        throw CaseError("unknown action '" + std::string(name)
                        + "'; valid actions are open, close, fault, clear");
      }

      inline SolverFile readSolverFile(const std::filesystem::path& path)
      {
        std::ifstream input(path);
        if (!input)
        {
          throw CaseError("solver file '" + path.string() + "': not found");
        }

        nlohmann::json root;
        try
        {
          root = nlohmann::json::parse(input);
        }
        catch (const nlohmann::json::parse_error& ex)
        {
          throw CaseError("solver file '" + path.string() + "': malformed JSON: "
                          + std::string(ex.what()));
        }

        if (!root.is_object())
        {
          Detail::throwCase("solver file", "expected object");
        }
        rejectUnknownKeys(root,
                          {"format_version", "case_file", "solve", "output", "schedule", "validation"},
                          "solver file");

        SolverFile file;
        file.path = path;

        file.format_version = require<int>(root, "format_version", "solver file");
        if (file.format_version != 1)
        {
          throw CaseError("format_version: unsupported version "
                          + std::to_string(file.format_version));
        }

        const auto base_dir = path.parent_path();
        file.case_file      = Detail::readPath(root, "case_file", "solver file", base_dir);

        const auto& solve = Detail::requireObject(root, "solve", "solver file");
        rejectUnknownKeys(solve,
                          {"t0", "tmax", "dt", "rel_tol", "abs_tol", "max_steps", "use_jacobian"},
                          "solve");
        file.solve.t0           = Detail::optionalValue<double>(solve, "t0", 0.0, "solve");
        file.solve.tmax         = require<double>(solve, "tmax", "solve");
        file.solve.dt           = require<double>(solve, "dt", "solve");
        file.solve.rel_tol      = Detail::optionalValue<double>(solve, "rel_tol", 1.0e-8, "solve");
        file.solve.abs_tol      = Detail::optionalValue<double>(solve, "abs_tol", 1.0e-8, "solve");
        file.solve.max_steps    = solve.contains("max_steps")
                                      ? Detail::readMaxSteps(solve, "solve")
                                      : std::size_t{200000};
        file.solve.use_jacobian = Detail::optionalValue<bool>(solve, "use_jacobian", true, "solve");

        Detail::requireFiniteNonnegative(file.solve.t0, "solve.t0");
        Detail::requireFinitePositive(file.solve.tmax, "solve.tmax");
        Detail::requireFinitePositive(file.solve.dt, "solve.dt");
        Detail::requireFinitePositive(file.solve.rel_tol, "solve.rel_tol");
        Detail::requireFinitePositive(file.solve.abs_tol, "solve.abs_tol");
        if (file.solve.tmax <= file.solve.t0)
        {
          Detail::throwCase("solve.tmax", "must be greater than solve.t0");
        }

        if (auto output = Detail::optionalObject(root, "output", "solver file"))
        {
          rejectUnknownKeys(output->get(), {"monitor", "ida_stats"}, "output");

          if (output->get().contains("monitor"))
          {
            file.output.monitor = Detail::readPath(output->get(), "monitor", "output", base_dir);
          }

          if (output->get().contains("ida_stats"))
          {
            file.output.ida_stats = Detail::readPath(output->get(), "ida_stats", "output", base_dir);
          }
        }

        if (auto schedule = root.find("schedule"); schedule != root.end())
        {
          if (!schedule->is_array())
          {
            Detail::throwCase("schedule", "expected array");
          }
          file.schedule.reserve(schedule->size());
          for (std::size_t i = 0; i < schedule->size(); ++i)
          {
            const auto&       item   = (*schedule)[i];
            const std::string entity = "schedule[" + std::to_string(i) + "]";
            if (!item.is_object())
            {
              Detail::throwCase(entity, "expected object");
            }
            rejectUnknownKeys(item, {"time", "target", "action", "params"}, entity);

            Event event;
            event.time = require<double>(item, "time", entity);
            Detail::requireFiniteNonnegative(event.time, Detail::fieldContext(entity, "time"));
            if (event.time < file.solve.t0)
            {
              throw CaseError(entity + ".time: precedes solve.t0");
            }
            if (event.time > file.solve.tmax)
            {
              throw CaseError(entity + ".time: exceeds solve.tmax");
            }

            event.target = require<std::string>(item, "target", entity);
            if (event.target.empty())
            {
              Detail::throwCase(Detail::fieldContext(entity, "target"), "must not be empty");
            }

            const auto action_name = require<std::string>(item, "action", entity);
            const auto params_it   = item.find("params");
            const auto params      = params_it == item.end() ? Detail::emptyObject() : *params_it;
            event.action           = readAction(action_name, params);
            event.order            = i;
            file.schedule.push_back(std::move(event));
          }
        }

        if (auto validation = Detail::optionalObject(root, "validation", "solver file"))
        {
          rejectUnknownKeys(validation->get(), {"reference_file", "error_tolerance"}, "validation");

          Validation parsed;
          parsed.reference_file  = Detail::readPath(validation->get(), "reference_file", "validation", base_dir);
          parsed.error_tolerance = Detail::optionalValue<double>(validation->get(),
                                                                 "error_tolerance",
                                                                 1.0e-4,
                                                                 "validation");
          Detail::requireFinitePositive(parsed.error_tolerance, "validation.error_tolerance");
          file.validation = std::move(parsed);
        }

        return file;
      }

      template <class DataT>
      void installSchedule(Case<DataT>& emt_case, const std::vector<Event>& schedule)
      {
        for (const auto& event : schedule)
        {
          if (auto bus = emt_case.names.buses.find(event.target);
              bus != emt_case.names.buses.end())
          {
            std::visit(
                [&](const auto& action)
                {
                  emt_case.data.schedule(event.time, emt_case.data.busRef(bus->second), action);
                },
                event.action);
            continue;
          }

          if (auto component = emt_case.names.components.find(event.target);
              component != emt_case.names.components.end())
          {
            std::visit(
                [&](const auto& action)
                {
                  emt_case.data.schedule(event.time, component->second, action);
                },
                event.action);
            continue;
          }

          throw CaseError("unknown event target '" + event.target + "'");
        }
      }

      template <class DataT>
      std::optional<std::filesystem::path> monitorOutputFile(const DataT& data)
      {
        if (data.monitor_sinks.size() == 1u && !data.monitor_sinks.front().file_name.empty())
        {
          return std::filesystem::path{data.monitor_sinks.front().file_name};
        }
        return std::nullopt;
      }

      template <class DataT>
      void applyOutput(DataT& data, const Output& output)
      {
        if (!output.monitor.has_value())
        {
          return;
        }

        const auto file = output.monitor->string();
        if (data.monitor_sinks.empty())
        {
          data.addMonitorSink({file, GridKit::Model::VariableMonitorFormat::CSV, ","});
          return;
        }

        if (data.monitor_sinks.size() == 1u)
        {
          data.monitor_sinks.front().file_name = file;
          return;
        }

        throw CaseError("output.monitor is ambiguous because the case defines multiple monitor sinks");
      }

    } // namespace IO
  } // namespace EMT
} // namespace GridKit
