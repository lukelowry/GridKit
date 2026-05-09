/**
 * @file ThreeBusVoltageSourceJson.cpp
 * @brief EMT three-bus radial example using the shared JSON case parser.
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

namespace
{
  using scalar_type = double;
  using index_type  = size_t;
  using real_type   = double;
  using action_type = GridKit::EMT::Action;
  using json        = nlohmann::json;

  struct Cue
  {
    real_type   time{0.0};
    std::string target;
    action_type action{action_type::Off};
  };

  struct StudyData
  {
    int                   format_version{1};
    std::filesystem::path case_file{"ThreeBusVoltageSource.case.json"};
    real_type             dt{0.000125};
    real_type             tmax{0.04};
    std::vector<Cue>      schedule{{0.01, "load_extra", action_type::On}};
    std::filesystem::path output_file{"emt_three_bus_output.csv"};
  };

  auto resolvePath(const std::filesystem::path& base, const std::filesystem::path& path)
      -> std::filesystem::path
  {
    if (path.is_absolute())
    {
      return path;
    }
    return base / path;
  }

  auto isSolverFile(const std::filesystem::path& path) -> bool
  {
    return path.filename().string().find(".solver.") != std::string::npos;
  }

  auto readStudyFile(const std::filesystem::path& input_file) -> StudyData
  {
    std::ifstream input(input_file);
    if (!input)
    {
      throw std::runtime_error("Unable to open solver file: " + input_file.string());
    }

    json root;
    input >> root;

    const auto base = input_file.parent_path();
    StudyData  data;

    data.format_version = root.value("format_version", 1);
    if (data.format_version != 1)
    {
      throw std::runtime_error("Unsupported solver file format_version: "
                               + std::to_string(data.format_version));
    }

    if (!root.contains("case_file"))
    {
      throw std::runtime_error("Solver file is missing case_file");
    }
    data.case_file = resolvePath(base, root.at("case_file").get<std::string>());
    if (root.contains("output_file"))
    {
      data.output_file = resolvePath(base, root.at("output_file").get<std::string>());
    }
    if (root.contains("dt"))
    {
      data.dt = root.at("dt").get<real_type>();
    }
    if (root.contains("tmax"))
    {
      data.tmax = root.at("tmax").get<real_type>();
    }

    data.schedule.clear();
    if (root.contains("schedule"))
    {
      real_type last_time = -std::numeric_limits<real_type>::infinity();
      for (const auto& cue_input : root.at("schedule"))
      {
        Cue cue;
        cue.time   = cue_input.at("time").get<real_type>();
        cue.target = cue_input.at("target").get<std::string>();

        const auto action = cue_input.at("action").get<std::string>();
        if (action == "on")
        {
          cue.action = action_type::On;
        }
        else if (action == "off")
        {
          cue.action = action_type::Off;
        }
        else
        {
          throw std::runtime_error("Solver file: unknown action '" + action
                                   + "' (valid: on, off)");
        }

        if (cue.time > data.tmax)
        {
          throw std::runtime_error("Solver file: schedule cue exceeds tmax");
        }
        if (cue.time < last_time)
        {
          throw std::runtime_error("Solver file: schedule is not monotone non-decreasing in time");
        }

        last_time = cue.time;
        data.schedule.push_back(cue);
      }
    }

    return data;
  }

  void printUsage()
  {
    std::cout << "\n"
                 "ERROR: No input file found or provided.\n"
                 "\n"
                 "Usage:\n"
                 "       ThreeBusVoltageSourceJson <solver-or-case-json-file> [output-csv]\n"
                 "\n"
                 "By default this example looks for \"ThreeBusVoltageSource.solver.json\" in the\n"
                 "current working directory and uses that if found.\n"
                 "\n";
  }
} // namespace

int main(int argc, char** argv)
{
  using namespace AnalysisManager::Sundials;
  using namespace GridKit::EMT;

  std::filesystem::path input_file;
  if (argc < 2)
  {
    if (std::filesystem::exists("ThreeBusVoltageSource.solver.json"))
    {
      input_file = std::filesystem::current_path() / "ThreeBusVoltageSource.solver.json";
    }
    else if (std::filesystem::exists("ThreeBusVoltageSource.case.json"))
    {
      input_file = std::filesystem::current_path() / "ThreeBusVoltageSource.case.json";
    }
    else
    {
      printUsage();
      return 1;
    }
  }
  else
  {
    input_file = argv[1];
  }

  StudyData study;
  if (isSolverFile(input_file))
  {
    study = readStudyFile(input_file);
  }
  else
  {
    study.case_file   = input_file;
    study.output_file = input_file.parent_path() / "emt_three_bus_output.csv";
  }

  if (argc > 2)
  {
    study.output_file = argv[2];
  }

  auto data = parseSystemModelData(study.case_file);
  if (data.voltage_source.empty())
  {
    throw std::runtime_error("ThreeBusVoltageSourceJson requires a VoltageSource");
  }

  SystemModel<scalar_type, index_type> system(data);
  if (system.allocate() != 0)
  {
    throw std::runtime_error("Unable to allocate EMT three-bus model");
  }

  auto* source = system.getVoltageSource(data.voltage_source.front().id);
  if (source == nullptr)
  {
    throw std::runtime_error("Unable to find source '" + data.voltage_source.front().id + "'");
  }
  const auto source_current = source->variableRange().begin;

  auto* switched_load = system.getShuntLoad("load_extra");
  for (const auto& cue : study.schedule)
  {
    if (system.getShuntLoad(cue.target) == nullptr)
    {
      throw std::runtime_error("Unable to find cue target '" + cue.target + "'");
    }
  }

  if (!study.output_file.parent_path().empty())
  {
    std::filesystem::create_directories(study.output_file.parent_path());
  }
  std::ofstream output(study.output_file);
  if (!output)
  {
    throw std::runtime_error("Unable to open output file: " + study.output_file.string());
  }

  output << "t,v1a,v1b,v1c,v2a,v2b,v2c,v3a,v3b,v3c,isa,isb,isc,load_event_closed\n";
  output << std::setprecision(17);

  auto record = [&](real_type t)
  {
    const auto& y = system.y();
    output << t << ','
           << y[0] << ',' << y[1] << ',' << y[2] << ','
           << y[3] << ',' << y[4] << ',' << y[5] << ','
           << y[6] << ',' << y[7] << ',' << y[8] << ','
           << y[source_current + 0] << ','
           << y[source_current + 1] << ','
           << y[source_current + 2] << ','
           << (switched_load != nullptr && switched_load->status() ? 1 : 0) << '\n';
  };

  Ida<scalar_type, index_type> ida(&system);
  ida.configureSimulation();
  ida.initializeSimulation(0.0, true);

  real_type time = 0.0;
  record(time);

  auto runTo = [&](real_type target_time)
  {
    if (target_time <= time)
    {
      return;
    }
    const auto nout = std::max(1, static_cast<int>(std::round((target_time - time) / study.dt)));
    ida.runSimulation(target_time, nout, record);
    time = target_time;
  };

  size_t cue_index = 0;
  while (cue_index < study.schedule.size())
  {
    const auto cue_time = study.schedule[cue_index].time;
    if (cue_time < time || cue_time > study.tmax)
    {
      throw std::runtime_error("EMT cue time is outside the simulated interval");
    }

    runTo(cue_time);
    while (cue_index < study.schedule.size()
           && study.schedule[cue_index].time == cue_time)
    {
      const auto& cue = study.schedule[cue_index];
      system.cue(cue.target, cue.action);
      ++cue_index;
    }
    ida.initializeSimulation(cue_time, true);
    record(cue_time);
  }
  runTo(study.tmax);

  std::cout << "Example: ThreeBusVoltageSourceJson\n";
  std::cout << "Input file: " << study.case_file << '\n';
  std::cout << "Output file: " << study.output_file << '\n';

  return 0;
}
