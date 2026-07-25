#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/VariableMonitor.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit::EMT
{
  namespace fs = std::filesystem;

  using Log  = ::GridKit::Utilities::Logger;
  using json = nlohmann::json;

  struct SignalEvent
  {
    double      time{0.0};
    std::size_t signal_id{0};
    double      value{0.0};
  };

  struct StudyData
  {
    fs::path    system_model_file;
    double      dt_monitor{0.0};
    double      tmax{0.0};
    double      rel_tol{1.0e-7};
    double      abs_tol{1.0e-9};
    double      dt_fixed{0.0};
    std::size_t max_steps{0};
    bool        suppress_algebraic_errors{false};
    fs::path    output_file;

    std::vector<SignalEvent>             events;
    SystemModelData<double, std::size_t> model_data;
  };

  inline void validateJsonKeys(const json&                  object,
                               const std::set<std::string>& allowed,
                               const std::set<std::string>& required,
                               const std::string&           context)
  {
    if (!object.is_object())
    {
      throw std::runtime_error(context + " must be a JSON object");
    }
    for (const auto& [key, value] : object.items())
    {
      static_cast<void>(value);
      if (!allowed.contains(key))
      {
        throw std::runtime_error("Unknown " + context + " key: " + key);
      }
    }
    for (const auto& key : required)
    {
      if (!object.contains(key))
      {
        throw std::runtime_error("Missing " + context + " key: " + key);
      }
    }
  }

  inline std::size_t nonnegativeInteger(const json&        value,
                                        const std::string& context)
  {
    if (value.is_number_unsigned())
    {
      const auto parsed = value.get<std::uint64_t>();
      if (parsed
          > static_cast<std::uint64_t>(
              std::numeric_limits<long int>::max()))
      {
        throw std::runtime_error(
            context + " exceeds the SUNDIALS long-int range");
      }
      return static_cast<std::size_t>(parsed);
    }
    if (value.is_number_integer())
    {
      const auto parsed = value.get<std::int64_t>();
      if (parsed < 0)
      {
        throw std::runtime_error(context + " must be nonnegative");
      }
      if (parsed > std::numeric_limits<long int>::max())
      {
        throw std::runtime_error(
            context + " exceeds the SUNDIALS long-int range");
      }
      return static_cast<std::size_t>(parsed);
    }
    throw std::runtime_error(context + " must be an integer");
  }

  inline void from_json(const json& j, SignalEvent& event)
  {
    validateJsonKeys(j,
                     {"time", "type", "signal_id", "value"},
                     {"time", "type", "signal_id", "value"},
                     "EMT event");
    const auto type = j.at("type").get<std::string>();
    if (type != "signal_set")
    {
      throw std::runtime_error("EMT event type must be signal_set");
    }

    j.at("time").get_to(event.time);
    j.at("signal_id").get_to(event.signal_id);
    j.at("value").get_to(event.value);

    if (!std::isfinite(event.time) || event.time < 0.0)
    {
      throw std::runtime_error("EMT event time must be finite and nonnegative");
    }
    if (!std::isfinite(event.value)
        || (event.value != 0.0 && event.value != 1.0))
    {
      throw std::runtime_error("EMT load-enable event value must be 0 or 1");
    }
  }

  inline void from_json(const json& j, StudyData& study)
  {
    validateJsonKeys(j,
                     {"system_model_file",
                      "dt_monitor",
                      "tmax",
                      "rel_tol",
                      "abs_tol",
                      "dt_fixed",
                      "max_steps",
                      "suppress_algebraic_errors",
                      "output_file",
                      "events"},
                     {"system_model_file", "tmax"},
                     "EMT study");
    j.at("system_model_file").get_to(study.system_model_file);
    j.at("tmax").get_to(study.tmax);
    study.dt_monitor  = j.value("dt_monitor", 0.0);
    study.rel_tol     = j.value("rel_tol", 1.0e-7);
    study.abs_tol     = j.value("abs_tol", 1.0e-9);
    study.dt_fixed    = j.value("dt_fixed", 0.0);
    study.output_file = j.value("output_file", fs::path{});

    if (j.contains("max_steps"))
    {
      study.max_steps = nonnegativeInteger(j.at("max_steps"),
                                           "EMT max_steps");
    }
    if (j.contains("suppress_algebraic_errors"))
    {
      if (!j.at("suppress_algebraic_errors").is_boolean())
      {
        throw std::runtime_error(
            "EMT suppress_algebraic_errors must be boolean");
      }
      study.suppress_algebraic_errors =
          j.at("suppress_algebraic_errors").get<bool>();
    }

    if (j.contains("events"))
    {
      j.at("events").get_to(study.events);
    }

    if (!std::isfinite(study.tmax) || study.tmax <= 0.0
        || !std::isfinite(study.dt_monitor) || study.dt_monitor < 0.0
        || !std::isfinite(study.dt_fixed) || study.dt_fixed < 0.0
        || !std::isfinite(study.rel_tol) || study.rel_tol <= 0.0
        || !std::isfinite(study.abs_tol) || study.abs_tol <= 0.0)
    {
      throw std::runtime_error("Invalid EMT solver settings");
    }

    std::ranges::stable_sort(study.events, {}, &SignalEvent::time);
    std::set<std::pair<double, std::size_t>> event_targets;
    for (const auto& event : study.events)
    {
      if (event.time >= study.tmax)
      {
        throw std::runtime_error("EMT event must occur before tmax");
      }
      if (!event_targets.emplace(event.time, event.signal_id).second)
      {
        throw std::runtime_error(
            "Duplicate EMT event time and signal_id");
      }
    }
  }

  inline StudyData parseStudyData(const fs::path& file_path)
  {
    std::ifstream input(file_path);
    if (!input)
    {
      throw std::runtime_error("Could not open EMT study file: "
                               + file_path.string());
    }

    StudyData  study           = json::parse(input).get<StudyData>();
    const auto input_directory = file_path.parent_path();

    if (!study.system_model_file.is_absolute())
    {
      study.system_model_file = input_directory / study.system_model_file;
    }
    if (!study.output_file.empty() && !study.output_file.is_absolute())
    {
      study.output_file = input_directory / study.output_file;
    }
    study.model_data = parseSystemModelData(study.system_model_file);

    std::set<std::size_t> load_enable_signals;
    for (const auto& load : study.model_data.loadz)
    {
      load_enable_signals.insert(
          load.signal_inputs.at(LoadZSignalInputs::enable));
    }
    for (const auto& event : study.events)
    {
      if (!load_enable_signals.contains(event.signal_id))
      {
        throw std::runtime_error(
            "EMT signal_set events may target only LoadZ enable signals");
      }
    }

    if (!study.output_file.empty())
    {
      study.model_data.monitor_sink.clear();
      study.model_data.monitor_sink.push_back({Model::VariableMonitorFormat::CSV,
                                               study.output_file.string(),
                                               ","});
    }

    return study;
  }
} // namespace GridKit::EMT
