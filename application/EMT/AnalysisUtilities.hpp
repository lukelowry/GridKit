#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace fs = std::filesystem;

    using json = nlohmann::json;

    struct BusFaultInterval
    {
      using FaultConductanceT = SystemModel<double, size_t, 3>::FaultConductanceT;

      size_t                bus{0};
      double                start_time{0.0};
      std::optional<double> clear_time;
      FaultConductanceT     G{};
    };

    struct BusFaultEvent
    {
      double time{0.0};
      size_t fault_index{0};
      bool   apply{false};
    };

    struct StudyData
    {
      using ModelDataT = SystemModelData<double, size_t, 3>;

      fs::path                      system_model_file;
      double                        dt{0.0};
      double                        tmax{0.0};
      fs::path                      output_file;
      double                        fixed_step{0.0};
      bool                          find_consistent{true};
      std::vector<BusFaultInterval> faults;
      ModelDataT                    model_data;
    };

    inline void from_json(const json& j, BusFaultInterval& data)
    {
      j.at("bus").get_to(data.bus);
      j.at("start_time").get_to(data.start_time);

      data.clear_time.reset();
      if (j.contains("clear_time"))
      {
        data.clear_time = j.at("clear_time").get<double>();
      }

      const auto& raw_G = j.at("G");
      if (!raw_G.is_array() || raw_G.size() != 3)
      {
        throw std::invalid_argument("EMT DynamicSimulation: fault G must be a 3x3 array");
      }

      for (std::size_t r = 0; r < 3; ++r)
      {
        if (!raw_G.at(r).is_array() || raw_G.at(r).size() != 3)
        {
          throw std::invalid_argument("EMT DynamicSimulation: fault G must be a 3x3 array");
        }

        for (std::size_t c = 0; c < 3; ++c)
        {
          data.G[r][c] = raw_G.at(r).at(c).get<double>();
        }
      }
    }

    inline void from_json(const json& j, StudyData& data)
    {
      j.at("system_model_file").get_to(data.system_model_file);
      j.at("dt").get_to(data.dt);
      j.at("tmax").get_to(data.tmax);

      if (j.contains("output_file"))
      {
        j.at("output_file").get_to(data.output_file);
      }

      if (j.contains("fixed_step"))
      {
        j.at("fixed_step").get_to(data.fixed_step);
      }

      if (j.contains("find_consistent"))
      {
        j.at("find_consistent").get_to(data.find_consistent);
      }

      data.faults.clear();
      if (j.contains("faults"))
      {
        j.at("faults").get_to(data.faults);
      }
    }

    inline bool approximatelyEqual(double x, double y)
    {
      const auto scale = std::max({1.0, std::abs(x), std::abs(y)});
      return std::abs(x - y) <= 1.0e-9 * scale;
    }

    inline bool timeAligned(double time, double dt)
    {
      const auto step = std::round(time / dt);
      return approximatelyEqual(time, step * dt);
    }

    inline bool hasBus(const StudyData::ModelDataT& data, size_t bus_id)
    {
      return std::any_of(data.bus.begin(),
                         data.bus.end(),
                         [bus_id](const auto& bus)
                         { return bus.bus_id == bus_id; });
    }

    inline void validateFaultConductance(const BusFaultInterval& fault)
    {
      constexpr double tol = 1.0e-12;

      for (std::size_t r = 0; r < 3; ++r)
      {
        double row_sum = 0.0;
        for (std::size_t c = 0; c < 3; ++c)
        {
          const auto value = fault.G[r][c];
          if (!std::isfinite(value))
          {
            throw std::invalid_argument("EMT DynamicSimulation: fault G contains a non-finite value");
          }

          if (!approximatelyEqual(value, fault.G[c][r]))
          {
            throw std::invalid_argument("EMT DynamicSimulation: fault G must be symmetric");
          }

          if (r == c && value < -tol)
          {
            throw std::invalid_argument("EMT DynamicSimulation: fault G diagonal entries must be nonnegative");
          }

          if (r != c && value > tol)
          {
            throw std::invalid_argument("EMT DynamicSimulation: fault G off-diagonal entries must be nonpositive");
          }

          row_sum += value;
        }

        if (row_sum < -tol)
        {
          throw std::invalid_argument("EMT DynamicSimulation: fault G row sums must be nonnegative");
        }
      }
    }

    inline void validateStudyData(const StudyData& data)
    {
      if (data.dt <= 0.0)
      {
        throw std::invalid_argument("EMT DynamicSimulation: dt must be positive");
      }

      if (data.tmax <= 0.0)
      {
        throw std::invalid_argument("EMT DynamicSimulation: tmax must be positive");
      }

      if (data.fixed_step < 0.0)
      {
        throw std::invalid_argument("EMT DynamicSimulation: fixed_step must be nonnegative");
      }

      for (const auto& fault : data.faults)
      {
        if (!hasBus(data.model_data, fault.bus))
        {
          std::stringstream ss;
          ss << "EMT DynamicSimulation: fault references unknown bus " << fault.bus;
          throw std::invalid_argument(ss.str());
        }

        auto validateTime = [&data](double time, const char* name)
        {
          if (time < 0.0 || time > data.tmax)
          {
            std::stringstream ss;
            ss << "EMT DynamicSimulation: fault " << name << " must be in [0, tmax]";
            throw std::invalid_argument(ss.str());
          }

          if (!timeAligned(time, data.dt))
          {
            std::stringstream ss;
            ss << "EMT DynamicSimulation: fault " << name << " must align with dt";
            throw std::invalid_argument(ss.str());
          }
        };

        validateTime(fault.start_time, "start_time");
        if (fault.clear_time.has_value())
        {
          validateTime(fault.clear_time.value(), "clear_time");
          if (fault.clear_time.value() <= fault.start_time)
          {
            throw std::invalid_argument("EMT DynamicSimulation: fault clear_time must be greater than start_time");
          }
        }

        validateFaultConductance(fault);
      }
    }

    inline std::vector<BusFaultEvent> busFaultEvents(const StudyData& data)
    {
      std::vector<BusFaultEvent> events;
      events.reserve(2 * data.faults.size());

      for (std::size_t k = 0; k < data.faults.size(); ++k)
      {
        const auto& fault = data.faults[k];
        events.push_back({fault.start_time, k, true});

        if (fault.clear_time.has_value())
        {
          events.push_back({fault.clear_time.value(), k, false});
        }
      }

      std::sort(events.begin(),
                events.end(),
                [](const auto& lhs, const auto& rhs)
                { return lhs.time < rhs.time; });

      return events;
    }

    inline fs::path defaultOutputFile(const fs::path& solver_file)
    {
      auto              stem   = solver_file.stem().string();
      const std::string suffix = ".solver";

      if (stem.size() > suffix.size()
          && stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        stem.erase(stem.size() - suffix.size());
      }

      return solver_file.parent_path() / (stem + ".csv");
    }

    inline std::ifstream openFile(const fs::path& file_path)
    {
      auto stream = std::ifstream(file_path);
      if (!stream)
      {
        std::stringstream ss;
        ss << "Failed to open file: " << file_path;
        throw std::runtime_error(ss.str());
      }

      return stream;
    }

    inline StudyData parseStudyData(const fs::path& file_path)
    {
      auto data = json::parse(openFile(file_path)).get<StudyData>();
      auto loc  = file_path.parent_path();

      if (!data.system_model_file.is_absolute())
      {
        data.system_model_file = loc / data.system_model_file;
      }

      if (data.output_file.empty())
      {
        data.output_file = defaultOutputFile(file_path);
      }
      else if (!data.output_file.is_absolute())
      {
        data.output_file = loc / data.output_file;
      }

      data.model_data = parseSystemModelData(data.system_model_file);
      validateStudyData(data);
      return data;
    }

    inline void checkCommandLine(int argc, const std::string& app_name)
    {
      if (argc >= 2)
      {
        return;
      }

      std::cerr << "\n"
                << "Usage:\n"
                << "       " << app_name << " <json-input-file>\n"
                << "\n"
                << "Please provide a json input file for the study to run.\n"
                << "\n";
      std::exit(1);
    }

    struct TraceChannel
    {
      std::string                                         name;
      std::function<double(SystemModel<double, size_t>&)> read;
    };

    class TraceWriter
    {
    public:
      TraceWriter(const fs::path& file_path, const StudyData::ModelDataT& data)
        : out_(file_path)
      {
        if (!out_)
        {
          std::stringstream ss;
          ss << "Failed to open output file: " << file_path;
          throw std::runtime_error(ss.str());
        }

        for (const auto& bus : data.bus)
        {
          const auto name = bus.name.empty() ? "bus" + std::to_string(bus.bus_id) : bus.name;

          for (size_t n = 0; n < bus.v0.size(); ++n)
          {
            const auto phase = static_cast<char>('a' + n);
            channels_.push_back({name + ".v_" + std::string(1, phase),
                                 [bus_id = bus.bus_id, n](SystemModel<double, size_t>& system)
                                 {
                                   const auto* typed_bus =
                                       dynamic_cast<const SystemModel<double, size_t>::BusT*>(system.getBus(bus_id));
                                   if (typed_bus == nullptr)
                                   {
                                     throw std::runtime_error("TraceWriter: EMT bus channel resolved to a non-EMT bus");
                                   }

                                   return typed_bus->V(n);
                                 }});
          }
        }

        for (size_t k = 0; k < data.line_lumped.size(); ++k)
        {
          const auto name         = data.line_lumped[k].disambiguation_string;
          const auto component_id = data.voltage_source.size() + data.load_rl.size() + k;
          for (size_t n = 0; n < 3; ++n)
          {
            const auto phase = static_cast<char>('a' + n);
            channels_.push_back({name + ".i_" + std::string(1, phase),
                                 [component_id, n](SystemModel<double, size_t>& system)
                                 {
                                   const auto* line =
                                       dynamic_cast<const SystemModel<double, size_t>::LineT*>(
                                           system.getComponent(component_id));
                                   if (line == nullptr)
                                   {
                                     throw std::runtime_error("TraceWriter: EMT line channel resolved to a non-line component");
                                   }

                                   return line->I(n);
                                 }});
          }
        }
      }

      void writeHeader()
      {
        out_ << "time";
        for (const auto& channel : channels_)
        {
          out_ << "," << channel.name;
        }
        out_ << "\n";
      }

      void write(double time, SystemModel<double, size_t>& system)
      {
        out_ << std::setprecision(17) << time;
        for (const auto& channel : channels_)
        {
          out_ << "," << channel.read(system);
        }
        out_ << "\n";
      }

    private:
      std::ofstream             out_;
      std::vector<TraceChannel> channels_;
    };
  } // namespace EMT
} // namespace GridKit
