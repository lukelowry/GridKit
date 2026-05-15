#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/IO/SolverFile.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Testing/CSV.hpp>

namespace
{
  using Real = double;
  using Idx  = std::size_t;
  using Data = GridKit::EMT::CaseData<Real, Idx>;
  using Ida  = AnalysisManager::Sundials::Ida<Real, Idx>;

  struct SolveResult
  {
    int                                 status{0};
    AnalysisManager::Sundials::IdaStats ida_summary;
    std::vector<GridKit::EMT::IO::IdaStatsSegment<AnalysisManager::Sundials::IdaStats>>
                                         ida_segments;
    std::optional<std::filesystem::path> monitor_file;
  };

  int outputSteps(Real start_time, Real end_time, Real dt)
  {
    if (end_time <= start_time)
    {
      return 0;
    }

    const auto raw = static_cast<long long>(std::llround((end_time - start_time) / dt));
    return static_cast<int>(std::max<long long>(1, raw));
  }

  std::optional<AnalysisManager::Sundials::IdaLogOptions> idaLogOptions(
      const GridKit::EMT::IO::Output& output)
  {
    if (!output.ida.has_value() || !output.ida->log.has_value())
    {
      return std::nullopt;
    }

    AnalysisManager::Sundials::IdaLogOptions options;
    options.file  = output.ida->log->file;
    options.level = output.ida->log->level == "error"
                        ? AnalysisManager::Sundials::IdaLogLevel::Error
                        : AnalysisManager::Sundials::IdaLogLevel::Warning;
    return options;
  }

  void recordIdaStatsSegment(SolveResult& result,
                             const Ida&   ida,
                             Real         start_time,
                             Real         end_time,
                             int          output_steps)
  {
    auto stats          = ida.getStats();
    result.ida_summary += stats;
    result.ida_segments.push_back({start_time, end_time, output_steps, std::move(stats)});
  }

  SolveResult solveCase(Data                            data,
                        const GridKit::EMT::IO::Solve&  solve,
                        const GridKit::EMT::IO::Output& output)
  {
    SolveResult result;
    result.monitor_file = GridKit::EMT::IO::monitorOutputFile(data);

    GridKit::EMT::SystemModel<Data> system(std::move(data),
                                           solve.rel_tol,
                                           solve.abs_tol,
                                           solve.use_jacobian,
                                           static_cast<Idx>(solve.max_steps));
    system.allocate();

    Ida ida(&system, idaLogOptions(output));
    ida.configureSimulation();
    ida.initializeSimulation(solve.t0, false);
    system.updateTime(solve.t0, 0.0);
    system.printMonitoredVariables();

    Real current_time = solve.t0;
    while (result.status == 0)
    {
      const auto event_time = system.nextEventTime();
      if (!event_time || *event_time > solve.tmax)
      {
        break;
      }

      const int segment_steps = outputSteps(current_time, *event_time, solve.dt);
      if (segment_steps > 0)
      {
        result.status = ida.runSimulation(*event_time, segment_steps);
        recordIdaStatsSegment(result, ida, current_time, *event_time, segment_steps);
      }
      if (result.status != 0)
      {
        break;
      }

      system.applyNextEventBatch();
      ida.initializeSimulation(*event_time, false);
      system.updateTime(*event_time, 0.0);
      system.printMonitoredVariables();
      current_time = *event_time;
    }

    if (result.status == 0)
    {
      const int segment_steps = outputSteps(current_time, solve.tmax, solve.dt);
      if (segment_steps > 0)
      {
        result.status = ida.runSimulation(solve.tmax, segment_steps);
        recordIdaStatsSegment(result, ida, current_time, solve.tmax, segment_steps);
      }
    }

    system.stopMonitor();
    return result;
  }

  bool validateResult(const SolveResult& result, const GridKit::EMT::IO::Validation& validation)
  {
    if (!result.monitor_file.has_value())
    {
      throw GridKit::EMT::CaseError("validation requires a monitor output file");
    }

    auto errors = GridKit::Testing::compareCSV(result.monitor_file->string(),
                                               validation.reference_file.string());
    errors.display();
    return errors.total.max_error < validation.error_tolerance;
  }
} // namespace

int main(int argc, const char* argv[])
{
  if (argc != 2)
  {
    std::cerr << "Usage: EMTSim <solver-file.solver.json>\n";
    return EXIT_FAILURE;
  }

  try
  {
    const auto file     = GridKit::EMT::IO::readSolverFile(argv[1]);
    auto       emt_case = GridKit::EMT::loadCase<Data>(file.case_file);

    GridKit::EMT::IO::applyOutput(emt_case.data, file.output);
    GridKit::EMT::IO::installSchedule(emt_case, file.schedule);

    const auto result = solveCase(std::move(emt_case.data), file.solve, file.output);

    if (file.output.ida.has_value())
    {
      GridKit::EMT::IO::writeIdaStats(result.ida_summary, result.ida_segments, *file.output.ida);
    }

    bool validation_passed = true;
    if (result.status == 0 && file.validation.has_value())
    {
      validation_passed = validateResult(result, *file.validation);
    }

    return result.status == 0 && validation_passed ? EXIT_SUCCESS : EXIT_FAILURE;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "EMTSim error: " << ex.what() << '\n';
    return EXIT_FAILURE;
  }
}
