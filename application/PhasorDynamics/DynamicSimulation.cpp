#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include <GridKit/Model/PhasorDynamics/BusFault/BusFault.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "AnalysisUtilities.hpp"

using Log = GridKit::Utilities::Logger;

using namespace GridKit::PhasorDynamics;
using namespace GridKit::Testing;
using namespace AnalysisManager::Sundials;

using scalar_type = double;
using real_type   = double;
using index_type  = size_t;

int main(int argc, const char* argv[])
{
  // Study file
  checkCommandLine(argc, "DynamicSimulation");
  auto study = parseStudyData(argv[1]);

  if (study.fault_bus && !study.model_data.bus_fault.empty())
  {
    study.model_data.bus_fault.front().buses[BusFaultBuses::bus] = *study.fault_bus;
  }

  // Instantiate system
  SystemModel<scalar_type, index_type> sys(study.model_data);
  sys.allocate();

  std::cout << "\nGRIDKIT_SYSTEM_BEGIN\n"
            << "buses=" << study.model_data.bus.size() << '\n'
            << "states=" << sys.size() << '\n'
            << "jacobian_nnz=" << sys.nnz() << '\n'
            << "GRIDKIT_SYSTEM_END\n";

  // Set up simulation
  Ida<scalar_type, index_type> ida(&sys);
  IdaStatsRecorder             ida_stats_recorder(!study.ida_stats.empty());
  IdaStepHistoryRecorder       ida_steps_recorder(!study.ida_steps.empty());
  ida.setOptions(study.ida);
  ida.configureSimulation();
  ida.enableStepTrace(!study.step_trace_file.empty());

  // Start timer
  real_type start = static_cast<real_type>(clock());

  using EventType = SystemEvent::Type;

  // Initilize simultation for first run
  // A negative dt_monitor in the study file requests output at
  // solver-selected steps, passed to IDA as an empty monitor interval.
  const std::optional<real_type> dt_monitor = study.dt_monitor < 0.0
                                                  ? std::optional<real_type>{}
                                                  : std::optional<real_type>{study.dt_monitor};
  real_type                      final_time = study.tmax;
  IdaStats                       stats;
  int                            segment = 0;
  ida.initializeSimulation(0.0);

  real_type                  curr_time    = 0.0;
  int                        solve_status = 0;
  std::exception_ptr         pending_exception;
  std::optional<std::string> pending_what;
  auto                       monitor_steps = [dt_monitor](real_type start_time, real_type end_time)
  {
    if (!dt_monitor.has_value())
    {
      return 0;
    }
    return *dt_monitor > 0.0 ? static_cast<int>(std::round((end_time - start_time) / *dt_monitor)) : 1;
  };
  auto run_segment = [&](real_type start_time, real_type end_time)
  {
    ida.setTraceSegment(segment++);
    ida_stats_recorder.beginSegment(ida);
    ida_steps_recorder.beginSegment(ida, start_time, end_time, monitor_steps(start_time, end_time));
    try
    {
      if (ida_steps_recorder.enabled())
      {
        solve_status = ida.runSimulationWithStepHistory(end_time,
                                                        dt_monitor,
                                                        [&](const IdaStats& step_stats)
                                                        {
                                                          ida_steps_recorder.recordStep(step_stats);
                                                        });
      }
      else
      {
        solve_status = ida.runSimulation(end_time, dt_monitor);
      }
    }
    catch (const std::exception& ex)
    {
      solve_status      = -1;
      pending_exception = std::current_exception();
      pending_what      = std::string(ex.what());
    }
    // Capture diagnostics even if the segment failed — IdaGetX counters remain valid.
    ida_steps_recorder.endSegment(ida);
    ida_stats_recorder.endSegment(ida, start_time, end_time, monitor_steps(start_time, end_time));
    stats += ida.getStats();
  };

  // Update system time and record initial state
  sys.updateTime(0.0, 0.0);
  sys.printMonitoredVariables();

  for (const auto& event : study.events)
  {
    // Run to event time
    if (event.time > curr_time)
    {
      run_segment(curr_time, event.time);
    }
    if (solve_status != 0)
    {
      break;
    }

    // Set up run for event (to start at event time)
    switch (event.type)
    {
    case EventType::FAULT_ON:
      sys.getBusFault(0)->setStatus(true);
      break;
    case EventType::FAULT_OFF:
      sys.getBusFault(0)->setStatus(false);
      break;
    }

    // Re-initialize simulation at event time
    ida.initializeSimulation(event.time);
    curr_time = event.time;

    // Record post-event state
    sys.updateTime(event.time, 0.0);
    sys.printMonitoredVariables();
  }

  // Run to final time
  if (solve_status == 0 && final_time > curr_time)
  {
    run_segment(curr_time, final_time);
  }

  real_type stop = static_cast<real_type>(clock());

  // Stop the variable monitor
  sys.stopMonitor();

  if (!study.step_trace_file.empty())
  {
    writeStepTrace(study.step_trace_file, ida.getStepTrace());
  }
  if (!study.ida_stats.empty())
  {
    writeIdaStatsJson(ida_stats_recorder.report(), {study.ida_stats, std::nullopt});
  }
  if (!study.ida_steps.empty())
  {
    writeIdaStepHistoryJson(ida_steps_recorder.report(), {study.ida_steps, std::nullopt});
  }

  // Preserve original behaviour: surface the SUNDIALS exception (and its message)
  // by re-throwing now that diagnostics have been flushed to disk.
  if (pending_exception)
  {
    if (pending_what.has_value())
    {
      Log::error() << *pending_what << std::endl;
    }
    std::rethrow_exception(pending_exception);
  }

  // Generate aggregate errors comparing variable output to reference solution
  TestStatus status = checkErrors(study);

  // Report run time
  std::cout << "\n\nComplete in " << (stop - start) / CLOCKS_PER_SEC << " seconds\n";
  std::cout << '\n'
            << stats.report() << '\n';
  ida.printPerformanceStats();
  sys.printResidualPerformanceStats();

  return solve_status == 0 ? status.get() : solve_status;
}
