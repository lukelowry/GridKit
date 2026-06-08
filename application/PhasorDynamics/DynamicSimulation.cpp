#include <ctime>
#include <exception>
#include <iostream>
#include <optional>
#include <utility>

#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>

#include "AnalysisUtilities.hpp"
#include "StudyRunner.hpp"

using Log = GridKit::Utilities::Logger;

using namespace GridKit::PhasorDynamics;
using namespace AnalysisManager::Sundials;

using scalar_type = double;
using index_type  = size_t;

int main(int argc, const char* argv[])
{
  checkCommandLine(argc, "DynamicSimulation");

  auto study = parseStudyData(argv[1]);

  StudyRunOptions options;
  options.record_stats = !study.ida_stats.empty();
  options.record_steps = !study.ida_steps.empty();

  // Time the run with the CPU clock to preserve the original console report.
  const double cpu_start = static_cast<double>(std::clock());
  auto         result    = runDynamicStudy<scalar_type, index_type>(study, std::move(options));
  const double cpu_stop  = static_cast<double>(std::clock());

  // A configuration rejected before the run starts returns immediately.
  if (result.solve_status == 1)
  {
    return result.solve_status;
  }

  if (!study.ida_stats.empty())
  {
    writeIdaStatsJson(
        result.stats,
        {study.ida_stats, std::nullopt, "dynamic_simulation", result.wall_clock_seconds, result.config});
  }
  if (!study.ida_steps.empty())
  {
    writeIdaStepHistoryJson(
        result.steps,
        {study.ida_steps, std::nullopt, "dynamic_simulation", result.wall_clock_seconds, result.config});
  }

  // Surface any captured SUNDIALS exception now that diagnostics are flushed.
  if (result.pending_exception)
  {
    if (result.pending_what.has_value())
    {
      Log::error() << *result.pending_what << std::endl;
    }
    std::rethrow_exception(result.pending_exception);
  }

  // Generate aggregate errors comparing variable output to reference solution.
  auto status = checkErrors(study);

  // Report run time.
  std::cout << "\n\nComplete in " << (cpu_stop - cpu_start) / CLOCKS_PER_SEC << " seconds\n";

  return result.solve_status == 0 ? status.get() : result.solve_status;
}
