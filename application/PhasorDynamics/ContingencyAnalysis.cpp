#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <GridKit/Model/PhasorDynamics/BusFault/BusFault.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

#include "AnalysisUtilities.hpp"

using Clock = std::chrono::high_resolution_clock;
using Dur   = std::chrono::duration<double>;

using Log = GridKit::Utilities::Logger;

using namespace AnalysisManager::Sundials;
using namespace GridKit::PhasorDynamics;
using namespace GridKit::Testing;

using scalar_type = double;
using real_type   = double;
using index_type  = size_t;

namespace
{
  /// Serializes the logger, which the worker threads share.
  std::mutex log_mutex;
} // namespace

TestStatus runStudy(const StudyData& study_data)
{
  // Instantiate system
  SystemModel<scalar_type, index_type> sys(study_data.model_data);
  sys.allocate();

  // Set up simulation
  Ida<scalar_type, index_type> ida(&sys);
  ida.setOptions(study_data.ida);
  ida.configureSimulation();

  using EventType = SystemEvent::Type;

  // Initilize simultation for first run. The pre-fault point still needs the
  // consistent-initial-condition solve; skipping it leaves cases whose model
  // initialization is not already DAE-consistent failing at t = 0.
  real_type dt_monitor = study_data.dt_monitor;
  real_type final_time = study_data.tmax;
  ida.initializeSimulation(0.0);

  for (const auto& event : study_data.events)
  {
    // Run to event time
    ida.runSimulation(event.time, dt_monitor);

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
    ida.initializeSimulation(event.time, true);
  }

  // Run to final time
  ida.runSimulation(final_time, dt_monitor);

  // Stop the variable monitor
  sys.stopMonitor();

  return checkErrors(study_data, false);
}

/**
 * @brief Run one contingency, with the study's fault placed on `bus_id`.
 *
 * The case supplies a single fault device as a template and every contingency
 * relocates it, so a sweep needs no per-bus fault declared in the case file.
 *
 * @param[in,out] study_data - The caller's own copy, mutated in place and
 *                             reusable for the next bus it takes.
 * @param[in] sink_names - Sink file names as the case declared them, used as
 *                         the stem for this contingency's own output files.
 */
TestStatus singleFaultStudy(StudyData&                      study_data,
                            const std::vector<std::string>& sink_names,
                            index_type                      bus_id)
{
  study_data.model_data.bus_fault.front().buses[BusFaultBuses::bus] = bus_id;

  // Make distinct output files
  for (std::size_t i = 0; i < study_data.model_data.monitor_sink.size(); ++i)
  {
    auto path                                       = std::filesystem::path(sink_names[i]);
    study_data.model_data.monitor_sink[i].file_name = path.stem().string()
                                                      + "_bus" + std::to_string(bus_id)
                                                      + path.extension().string();
  }

  try
  {
    return runStudy(study_data);
  }
  catch (...)
  {
    const std::lock_guard<std::mutex> guard(log_mutex);
    Log::warning() << "exception caught at bus: " << bus_id << std::endl;
    return {false};
  }
}

/**
 * @brief Take contingencies off the shared queue until it is empty.
 *
 * Each worker keeps one private copy of the study. The sweep mutates only the
 * fault location, so a copy is reused across every bus the worker takes and a
 * run holds one copy per worker rather than one per contingency.
 */
void runWorker(const StudyData&               prototype,
               const std::vector<index_type>& bus_ids,
               std::atomic<std::size_t>&      next,
               std::vector<TestStatus>&       stat_vec)
{
  StudyData study = prototype;

  std::vector<std::string> sink_names;
  sink_names.reserve(study.model_data.monitor_sink.size());
  for (const auto& sink : study.model_data.monitor_sink)
  {
    sink_names.push_back(sink.file_name);
  }

  for (std::size_t i = next++; i < bus_ids.size(); i = next++)
  {
    stat_vec[i] = singleFaultStudy(study, sink_names, bus_ids[i]);
  }
}

int main(int argc, const char* argv[])
{
  // Study file
  checkCommandLine(argc, "ContingencyAnalysis");
  auto study_data = parseStudyData(argv[1]);

  if (study_data.model_data.bus_fault.empty())
  {
    Log::error() << "ContingencyAnalysis: the case declares no bus fault for the "
                 << "sweep to relocate\n";
    return 1;
  }

  // One contingency per bus in the system.
  std::vector<index_type> bus_ids;
  bus_ids.reserve(study_data.model_data.bus.size());
  for (const auto& bus : study_data.model_data.bus)
  {
    bus_ids.push_back(bus.bus_id);
  }

  // Concurrency is bounded so that memory tracks the worker count rather than
  // the contingency count. An optional second argument overrides it.
  std::size_t workers = 1;
#if defined(GRIDKIT_ENABLE_THREADS)
  workers = std::thread::hardware_concurrency();
  if (workers == 0)
  {
    workers = 1;
  }
#endif
  if (argc > 2)
  {
    workers = static_cast<std::size_t>(std::stoul(argv[2]));
  }
  workers = std::max<std::size_t>(1, std::min(workers, bus_ids.size()));

  const auto start = Clock::now();

  auto                     stat_vec = std::vector<TestStatus>(bus_ids.size(), true);
  std::atomic<std::size_t> next{0};

  if (workers == 1)
  {
    runWorker(study_data, bus_ids, next, stat_vec);
  }
  else
  {
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (std::size_t w = 0; w < workers; ++w)
    {
      pool.emplace_back(runWorker,
                        std::cref(study_data),
                        std::cref(bus_ids),
                        std::ref(next),
                        std::ref(stat_vec));
    }
    for (auto& worker : pool)
    {
      worker.join();
    }
  }

  const auto stop = Clock::now();
  const auto dur  = std::chrono::duration<double>(stop - start);
  std::cout << "\n\nContingencies : " << bus_ids.size()
            << "\nWorkers       : " << workers
            << "\nComplete in " << dur << "\n";

  TestStatus status;
  for (std::size_t i = 0; i < stat_vec.size(); ++i)
  {
    status *= stat_vec[i];
    if (!stat_vec[i])
    {
      std::cout << "Study failed for bus: " << bus_ids[i] << '\n';
    }
  }

  return status.get();
}
