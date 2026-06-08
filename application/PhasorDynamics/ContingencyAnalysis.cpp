#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <nlohmann/json.hpp>

#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

#include "AnalysisUtilities.hpp"
#include "StudyRunner.hpp"

namespace fs = std::filesystem;

using Clock = std::chrono::high_resolution_clock;

using Log = GridKit::Utilities::Logger;

using namespace AnalysisManager::Sundials;
using namespace GridKit::PhasorDynamics;

using scalar_type   = double;
using index_type    = size_t;
using json          = nlohmann::json;
using BusFaultDataT = SystemModelData<scalar_type, index_type>::BusFaultDataT;

namespace
{
  constexpr const char* APP_NAME = "ContingencyAnalysis";

  std::atomic<std::size_t> next_worker_id{0};

  /// Per-fault outcome collected for the contingency summary.
  struct FaultResult
  {
    std::size_t fault_id{};
    index_type  bus_id{};
    std::string bus_name;
    bool        passed{false};
    int         solve_status{0};
    std::string solve_status_name;
    fs::path    ida_stats_path; ///< Relative to the results directory
  };

  /// Thread-safe terminal progress counter for the contingency sweep.
  class ProgressReporter
  {
  public:
    explicit ProgressReporter(std::size_t total)
      : total_(total),
        report_interval_(total > 20 ? total / 20 : 1),
        to_terminal_(::isatty(STDOUT_FILENO) != 0)
    {
    }

    void recordCompletion(bool passed)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      ++completed_;
      passed ? ++passed_ : ++failed_;
      if (to_terminal_)
      {
        std::cout << "\rContingencies: " << completed_ << "/" << total_ << " complete, " << passed_
                  << " passed, " << failed_ << " failed" << std::flush;
      }
      else if (completed_ == total_ || completed_ % report_interval_ == 0)
      {
        // Output is piped/redirected (no TTY): the live \r line never shows, so
        // emit newline-terminated milestones (~every 5%) instead. This lets a
        // tailed log report sweep progress rather than staying silent to the end.
        std::cout << "Contingencies: " << completed_ << "/" << total_ << " complete, " << passed_
                  << " passed, " << failed_ << " failed" << std::endl;
      }
    }

    void finish()
    {
      if (to_terminal_)
      {
        std::cout << '\n';
      }
    }

  private:
    std::size_t total_{};
    std::size_t report_interval_{1};
    std::size_t completed_{0};
    std::size_t passed_{0};
    std::size_t failed_{0};
    bool        to_terminal_{false};
    std::mutex  mutex_;
  };

  void forceFaultsOff(std::vector<BusFaultDataT>& faults)
  {
    for (auto& fault : faults)
    {
      fault.parameters[BusFaultParameters::state0] = false;
    }
  }

  void appendFile(std::ofstream& output, const fs::path& input_path)
  {
    if (input_path.empty() || !fs::exists(input_path))
    {
      return;
    }

    std::ifstream input(input_path, std::ios::binary);
    if (!input)
    {
      return;
    }
    output << input.rdbuf();
  }

  void concatenateLogs(const fs::path& prefix_log_path, const fs::path& suffix_log_path, const fs::path& final_log_path)
  {
    fs::create_directories(final_log_path.parent_path());
    std::ofstream output(final_log_path, std::ios::binary);
    if (!output)
    {
      throw std::runtime_error("failed to open contingency log output file '" + final_log_path.string() + "'");
    }
    appendFile(output, prefix_log_path);
    appendFile(output, suffix_log_path);
  }

  class ContingencyWorker
  {
  public:
    using Runner     = DynamicStudyRunner<scalar_type, index_type>;
    using Checkpoint = Runner::Checkpoint;

    ContingencyWorker(StudyData study, fs::path results_dir)
      : study_(std::move(study)),
        results_dir_(std::move(results_dir)),
        original_events_(study_.events),
        original_tmax_(study_.tmax),
        fault_count_(study_.model_data.bus_fault.size())
    {
      const std::size_t worker_id = next_worker_id.fetch_add(1);
      prefix_log_path_            = results_dir_ / ("worker_" + std::to_string(worker_id) + "_prefix.sundials.log");

      std::error_code ec;
      fs::remove(prefix_log_path_, ec);

      prefix_end_time_ = original_events_.empty() ? original_tmax_ : original_events_.front().time;
      study_.events.clear();
      study_.tmax = prefix_end_time_;

      StudyRunOptions options;
      options.record_stats = true;
      options.record_steps = false;
      options.log_options  = IdaLogOptions{prefix_log_path_, IdaLogLevel::Error};
      options.output_mode  = StudyOutputMode::SegmentEndOnly;

      runner_.emplace(study_, std::move(options));
      preparePrefixOnce();
    }

    ~ContingencyWorker()
    {
      std::error_code ec;
      fs::remove(prefix_log_path_, ec);
    }

    FaultResult runFault(std::size_t       fault_id,
                         index_type        bus_id,
                         std::string       bus_name,
                         ProgressReporter& progress)
    {
      FaultResult result;
      result.fault_id = fault_id;
      result.bus_id   = bus_id;
      result.bus_name = std::move(bus_name);

      const fs::path      fault_dir      = results_dir_ / ("fault_" + std::to_string(fault_id));
      const fs::path      ida_stats_path = idaStatsPath(fault_dir);
      const fs::path      final_log_path = fault_dir / "sundials.log";
      const IdaLogOptions final_log_options{final_log_path, IdaLogLevel::Error};
      result.ida_stats_path = fs::relative(ida_stats_path, results_dir_);

      try
      {
        fs::create_directories(fault_dir);

        if (!prefix_ready_)
        {
          auto stats = prefix_run_.stats;
          stats.log  = final_log_options;

          auto config              = prefix_run_.config;
          config.tmax              = original_tmax_;
          result.passed            = false;
          result.solve_status      = prefix_run_.solve_status;
          result.solve_status_name = prefix_run_.solve_status_name;

          concatenateLogs(prefix_log_path_, {}, final_log_path);
          writeIdaStatsJson(
              stats,
              {ida_stats_path, final_log_options, "contingency", prefix_run_.wall_clock_seconds, config});
          progress.recordCompletion(result.passed);
          return result;
        }

        setAllFaultsOff();

        const fs::path  suffix_log_path = fault_dir / "sundials.suffix.log";
        std::error_code ec;
        fs::remove(suffix_log_path, ec);

        const IdaLogOptions suffix_log_options{suffix_log_path, IdaLogLevel::Error};
        runner_->setLogger(suffix_log_options);

        const auto events     = eventsForFault(fault_id);
        auto       suffix_run = runner_->runFromCheckpoint(prefix_checkpoint_, events, original_tmax_);

        runner_->setLogger(std::nullopt);

        const auto   combined_stats     = combineIdaStatsReports(prefix_run_.stats, suffix_run.stats, final_log_options);
        const double wall_clock_seconds = prefix_run_.wall_clock_seconds + suffix_run.wall_clock_seconds;

        concatenateLogs(prefix_log_path_, suffix_log_path, final_log_path);
        fs::remove(suffix_log_path, ec);

        writeIdaStatsJson(
            combined_stats,
            {ida_stats_path, final_log_options, "contingency", wall_clock_seconds, suffix_run.config});

        result.solve_status      = suffix_run.solve_status;
        result.solve_status_name = suffix_run.solve_status_name;
        result.passed            = (suffix_run.solve_status == 0) && !suffix_run.pending_exception;
      }
      catch (const std::exception& ex)
      {
        try
        {
          runner_->setLogger(std::nullopt);
        }
        catch (...)
        {
        }
        Log::warning() << "exception caught at fault id: " << fault_id << " (" << ex.what() << ")" << std::endl;
        result.solve_status = -1;
        result.passed       = false;
      }

      progress.recordCompletion(result.passed);
      return result;
    }

  private:
    void preparePrefixOnce()
    {
      setAllFaultsOff();
      prefix_run_ = runner_->run();
      runner_->setLogger(std::nullopt);

      prefix_ready_ = (prefix_run_.solve_status == 0) && !prefix_run_.pending_exception;
      if (prefix_ready_)
      {
        prefix_checkpoint_ = runner_->saveCheckpoint(prefix_end_time_);
      }
    }

    void setAllFaultsOff()
    {
      for (std::size_t fault_id = 0; fault_id < fault_count_; ++fault_id)
      {
        runner_->system().getBusFault(static_cast<index_type>(fault_id))->setStatus(false);
      }
    }

    std::vector<SystemEvent> eventsForFault(std::size_t fault_id) const
    {
      auto events = original_events_;
      for (auto& event : events)
      {
        event.element_id = fault_id;
      }
      return events;
    }

    fs::path idaStatsPath(const fs::path& fault_dir) const
    {
      if (study_.ida_stats.empty())
      {
        return fault_dir / "ida_stats.json";
      }
      return resultFilePath(fault_dir, study_.ida_stats);
    }

    StudyData                study_;
    fs::path                 results_dir_;
    std::vector<SystemEvent> original_events_;
    double                   original_tmax_{0.0};
    double                   prefix_end_time_{0.0};
    std::size_t              fault_count_{0};
    fs::path                 prefix_log_path_;
    std::optional<Runner>    runner_;
    Checkpoint               prefix_checkpoint_;
    StudyRunResult           prefix_run_;
    bool                     prefix_ready_{false};
  };

  void writeSummary(const fs::path&                 results_dir,
                    const StudyData&                study_data,
                    const std::vector<FaultResult>& results,
                    std::size_t                     passed_count,
                    double                          wall_clock_seconds)
  {
    json summary;
    summary["schema"]            = "gridkit.contingency_summary.v1";
    summary["app"]               = APP_NAME;
    summary["system_model_file"] = study_data.system_model_file.string();
    summary["fault_count"]       = results.size();
    summary["passed_count"]      = passed_count;
    summary["failed_count"]      = results.size() - passed_count;
    summary["all_passed"]        = (passed_count == results.size());
    summary["timing"]            = {{"wall_clock_seconds", wall_clock_seconds}, {"scope", "contingency_analysis"}};
    summary["results_dir"]       = (fs::path("results") / APP_NAME).generic_string();
    summary["faults"]            = json::array();
    for (const auto& result : results)
    {
      summary["faults"].push_back({{"fault_id", result.fault_id},
                                   {"bus_id", result.bus_id},
                                   {"bus_name", result.bus_name},
                                   {"passed", result.passed},
                                   {"solve_status", result.solve_status},
                                   {"solve_status_name", result.solve_status_name},
                                   {"ida_stats", result.ida_stats_path.generic_string()}});
    }

    const fs::path summary_path = results_dir / "summary.json";
    std::ofstream  stream(summary_path);
    if (!stream)
    {
      Log::error() << "failed to open contingency summary file '" << summary_path << "'" << std::endl;
      return;
    }
    stream << summary.dump(2) << '\n';
  }
} // namespace

int main(int argc, const char* argv[])
{
  checkCommandLine(argc, APP_NAME);
  auto study_data = parseStudyData(argv[1]);

  const fs::path results_dir = studyResultsDirectory(study_data, APP_NAME);
  fs::create_directories(results_dir);

  forceFaultsOff(study_data.model_data.bus_fault);
  disableVariableMonitoring(study_data.model_data);
  study_data.output_file.clear();
  study_data.ida_steps.clear();

  const auto&       faults   = study_data.model_data.bus_fault;
  const std::size_t n_faults = faults.size();

  // Resolve the connected bus id/name for each fault for the summary.
  std::unordered_map<index_type, std::string> bus_names;
  for (const auto& bus : study_data.model_data.bus)
  {
    bus_names[bus.bus_id] = bus.name;
  }
  std::vector<index_type>  fault_bus_ids(n_faults, 0);
  std::vector<std::string> fault_bus_names(n_faults);
  for (std::size_t i = 0; i < n_faults; ++i)
  {
    const auto port = faults[i].ports.find(BusFaultPorts::bus);
    if (port != faults[i].ports.end())
    {
      fault_bus_ids[i]   = port->second;
      const auto name    = bus_names.find(port->second);
      fault_bus_names[i] = (name != bus_names.end()) ? name->second : std::string{};
    }
  }

  ProgressReporter         progress(n_faults);
  std::vector<FaultResult> results(n_faults);

  const auto start = Clock::now();

#if defined(GRIDKIT_ENABLE_THREADS)
  const unsigned int hardware_threads = std::thread::hardware_concurrency();
  const std::size_t  worker_count =
      std::min(n_faults, static_cast<std::size_t>(hardware_threads == 0 ? 1 : hardware_threads));

  std::atomic<std::size_t>       next_fault{0};
  std::vector<std::future<void>> futures;
  futures.reserve(worker_count);
  for (std::size_t worker = 0; worker < worker_count; ++worker)
  {
    futures.emplace_back(std::async(std::launch::async,
                                    [&]()
                                    {
                                      ContingencyWorker contingency_worker(study_data, results_dir);
                                      while (true)
                                      {
                                        const std::size_t i = next_fault.fetch_add(1);
                                        if (i >= n_faults)
                                        {
                                          break;
                                        }
                                        results[i] = contingency_worker.runFault(
                                            i, fault_bus_ids[i], fault_bus_names[i], progress);
                                      }
                                    }));
  }
  for (auto& future : futures)
  {
    future.get();
  }
#elif defined(_OPENMP)
#pragma omp parallel
  {
    ContingencyWorker worker(study_data, results_dir);
#pragma omp for schedule(dynamic)
    for (std::size_t i = 0; i < n_faults; ++i)
    {
      results[i] = worker.runFault(i, fault_bus_ids[i], fault_bus_names[i], progress);
    }
  }
#else
  ContingencyWorker worker(study_data, results_dir);
  for (std::size_t i = 0; i < n_faults; ++i)
  {
    results[i] = worker.runFault(i, fault_bus_ids[i], fault_bus_names[i], progress);
  }
#endif
  const auto   stop               = Clock::now();
  const double wall_clock_seconds = std::chrono::duration<double>(stop - start).count();
  progress.finish();

  std::size_t passed_count = 0;
  for (const auto& result : results)
  {
    if (result.passed)
    {
      ++passed_count;
    }
  }
  const std::size_t failed_count = n_faults - passed_count;

  writeSummary(results_dir, study_data, results, passed_count, wall_clock_seconds);

  std::cout << APP_NAME << ": " << passed_count << "/" << n_faults << " contingencies converged ("
            << failed_count << " did not) in " << wall_clock_seconds << " s. Results in " << results_dir
            << std::endl;

  // The sweep ran to completion; per-contingency convergence is reported in
  // summary.json (all_passed / failed_count). A non-converging contingency is a
  // result, not a tool failure, so it does not change the exit code.
  return 0;
}
