/**
 * @file FrequencyResponse.cpp
 *
 * @brief Evaluate EMT line-parameter models over frequency.
 *
 */

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <GridKit/Model/EMT/Constants.hpp>
#include <GridKit/Model/EMT/Parameters/Overhead.hpp>
#include <GridKit/Model/EMT/Parameters/OverheadDataJSONParser.hpp>
#include <GridKit/Model/LogEvaluator.hpp>
#include <GridKit/Model/VariableMonitor.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

#include "FrequencyResponseJSONParser.hpp"

using scalar_type = double;
using index_type  = size_t;
using Clock       = std::chrono::high_resolution_clock;

namespace
{
  namespace fs = std::filesystem;

  using Log = GridKit::Utilities::Logger;

  int usage()
  {
    std::cout << "\n"
              << "Usage:\n"
              << "       FrequencyResponse <solver-json-file>\n"
              << "\n"
              << "Please provide a FrequencyResponse solver JSON file.\n"
              << "\n";
    return 1;
  }

  int runFrequencyResponse(const fs::path& solver_file)
  {
    using namespace AnalysisManager::Sundials;
    using namespace GridKit::EMT::Application;
    using namespace GridKit::EMT::Parameters;

    const auto spec = parseFrequencyResponseData(solver_file);
    auto       data = parseOverheadData<scalar_type, index_type>(spec.model);

    data.monitored_variables = spec.variables;
    data.monitor_sink        = {{GridKit::Model::VariableMonitorFormat::CSV,
                                 spec.output_file.string()}};

    constexpr scalar_type pi          = GridKit::EMT::Constants::pi<scalar_type>();
    const scalar_type     omega_start = 2.0 * pi * spec.frequency.start;
    const scalar_type     omega_stop  = 2.0 * pi * spec.frequency.stop;

    Overhead<scalar_type, index_type>                     model(data);
    GridKit::Model::LogEvaluator<scalar_type, index_type> log_model(model, omega_start);
    log_model.allocate();

    Ida<scalar_type, index_type> ida(&log_model);
    ida.setOptions(spec.ida);
    ida.configureSimulation();
    ida.initializeSimulation(std::log(omega_start), true);

    log_model.printMonitoredVariables();
    ida.runSimulation(std::log(omega_stop), static_cast<int>(spec.frequency.points - 1));
    log_model.stopMonitor();

    return 0;
  }
} // namespace

int main(int argc, const char* argv[])
{
  if (argc != 2)
  {
    return usage();
  }

  const auto start = Clock::now();
  try
  {
    const int  retval = runFrequencyResponse(argv[1]);
    const auto stop   = Clock::now();
    const auto dur    = std::chrono::duration<double>(stop - start);
    std::cout << "\n\nComplete in " << dur << "\n";
    return retval;
  }
  catch (const std::exception& e)
  {
    Log::error() << e.what() << std::endl;
    return 1;
  }
}
