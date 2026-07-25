#include <ctime>
#include <exception>
#include <iostream>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

#include "AnalysisUtilities.hpp"

using ScalarT = double;
using IdxT    = std::size_t;

int main(int argc, const char* argv[])
{
  if (argc != 2)
  {
    std::cerr << "Usage: EMTDynamicSimulation <solver-json>\n";
    return 1;
  }

  try
  {
    auto study = GridKit::EMT::parseStudyData(argv[1]);

    GridKit::EMT::SystemModel<ScalarT, IdxT> system(study.model_data);
    system.allocate();

    AnalysisManager::Sundials::Ida<ScalarT, IdxT> ida(&system);
    ida.setTolerance(study.rel_tol, study.abs_tol);
    ida.setFixedStep(study.dt_fixed);
    ida.setMaxSteps(study.max_steps);
    ida.setSuppressAlgebraicErrors(study.suppress_algebraic_errors);
    if (ida.configureSimulation() != 0)
    {
      return 1;
    }

    const auto start = std::clock();
    if (ida.initializeSimulation(0.0, false) != 0)
    {
      return 1;
    }

    long int internal_steps{0};
    for (const auto& event : study.events)
    {
      if (ida.runSimulation(event.time, study.dt_monitor) != 0)
      {
        return 1;
      }
      internal_steps += ida.getStats().num_steps_;

      system.getSignal(event.signal_id)->init(event.value);
      if (ida.initializeSimulation(event.time) != 0)
      {
        return 1;
      }
    }

    if (ida.runSimulation(study.tmax, study.dt_monitor) != 0)
    {
      return 1;
    }
    internal_steps += ida.getStats().num_steps_;

    system.stopMonitor();
    const auto stop = std::clock();
    std::cout << "IDA internal steps: " << internal_steps << '\n';
    std::cout << "Complete in "
              << static_cast<double>(stop - start) / CLOCKS_PER_SEC
              << " seconds\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "EMTDynamicSimulation failed: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
