#pragma once

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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

    struct StudyData
    {
      using ModelDataT = SystemModelData<double, size_t, 3>;

      fs::path   system_model_file;
      double     dt{0.0};
      double     tmax{0.0};
      fs::path   output_file;
      double     fixed_step{0.0};
      bool       find_consistent{true};
      ModelDataT model_data;
    };

    using json = nlohmann::json;

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

      data.model_data = parseSystemModelData(data.system_model_file);
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
      std::string name;
      size_t      index;
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

        size_t offset = 0;
        for (const auto& bus : data.bus)
        {
          const auto name = bus.name.empty() ? "bus" + std::to_string(bus.bus_id) : bus.name;

          channels_.push_back({name + ".v_a", offset + 0});
          channels_.push_back({name + ".v_b", offset + 1});
          channels_.push_back({name + ".v_c", offset + 2});

          offset += bus.v0.size();
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

      void write(double time, const SystemModel<double, size_t>& system)
      {
        const auto& state = system.y();

        out_ << std::setprecision(17) << time;
        for (const auto& channel : channels_)
        {
          if (channel.index >= state.size())
          {
            std::stringstream ss;
            ss << "Trace channel \"" << channel.name << "\" index " << channel.index
               << " is outside the system state";
            throw std::runtime_error(ss.str());
          }
          out_ << "," << state[channel.index];
        }
        out_ << "\n";
      }

    private:
      std::ofstream             out_;
      std::vector<TraceChannel> channels_;
    };
  } // namespace EMT
} // namespace GridKit
