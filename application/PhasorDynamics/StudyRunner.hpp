#pragma once

#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <sundials/sundials_config.h>

#include <GridKit/Model/PhasorDynamics/BusFault/BusFault.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>

#include "AnalysisUtilities.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    /**
     * @brief Outcome of executing a single dynamic study (one fault scenario).
     */
    struct StudyRunResult
    {
      int                                             solve_status{0};
      std::string                                     solve_status_name;
      AnalysisManager::Sundials::IdaStatsReport       stats;
      AnalysisManager::Sundials::IdaStepHistoryReport steps;
      AnalysisManager::Sundials::IdaRunConfig         config;
      double                                          wall_clock_seconds{0.0};
      std::exception_ptr                              pending_exception;
      std::optional<std::string>                      pending_what;
    };

    enum class StudyOutputMode
    {
      StudyCadence,   ///< Use the study dt/output cadence.
      SegmentEndOnly, ///< Request only each segment endpoint.
    };

    struct StudyRunOptions
    {
      bool                                                    record_stats{false};
      bool                                                    record_steps{false};
      std::optional<AnalysisManager::Sundials::IdaLogOptions> log_options{};
      StudyOutputMode                                         output_mode{StudyOutputMode::StudyCadence};
    };

    namespace detail
    {
      inline bool validateStudyConfiguration(const StudyData& study, StudyRunResult& result)
      {
        if (study.dt < 0.0)
        {
          Log::error() << "dt must be nonnegative" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.ida_max_order.has_value() && (study.ida_max_order.value() < 1 || study.ida_max_order.value() > 5))
        {
          Log::error() << "ida_max_order must be between 1 and 5" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.ida_max_dt.has_value() && study.ida_max_dt.value() <= 0.0)
        {
          Log::error() << "ida_max_dt must be positive" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.ida_max_steps.has_value() && study.ida_max_steps.value() <= 0)
        {
          Log::error() << "ida_max_steps must be positive" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (!(study.mu > 0.0 && std::isfinite(study.mu)))
        {
          Log::error() << "mu must be a positive finite number" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.rel_tol.has_value() && !(study.rel_tol.value() > 0.0 && std::isfinite(study.rel_tol.value())))
        {
          Log::error() << "rel_tol must be a positive finite number" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.abs_tol.has_value() && !(study.abs_tol.value() > 0.0 && std::isfinite(study.abs_tol.value())))
        {
          Log::error() << "abs_tol must be a positive finite number" << std::endl;
          result.solve_status = 1;
          return false;
        }
        if (study.rel_tol.has_value() != study.abs_tol.has_value())
        {
          Log::error() << "rel_tol and abs_tol must be set together" << std::endl;
          result.solve_status = 1;
          return false;
        }
        return true;
      }
    } // namespace detail

    /**
     * @brief Reusable dynamic study engine.
     *
     * The class owns one SystemModel and one IDA workspace for a mutable StudyData
     * reference. `run()` preserves the one-shot DynamicSimulation behavior, while
     * `runFromCheckpoint()` lets sweep drivers reuse the assembled model/solver
     * and resume from a previously saved solution state.
     */
    template <class ScalarT, typename IdxT>
    class DynamicStudyRunner
    {
    public:
      using Checkpoint = AnalysisManager::Sundials::IdaSolutionCheckpoint<ScalarT>;

      DynamicStudyRunner(StudyData& study, StudyRunOptions options)
        : study_(study),
          options_(std::move(options)),
          sys_(study.model_data),
          ida_(&sys_, options_.log_options)
      {
        StudyRunResult validation_result;
        if (!detail::validateStudyConfiguration(study_, validation_result))
        {
          throw std::invalid_argument("invalid dynamic study configuration");
        }
        sys_.setMu(static_cast<typename GridKit::ScalarTraits<ScalarT>::RealT>(study_.mu));
        sys_.allocate();
        ida_.configureSimulation();
        applySolverOptions();
      }

      StudyRunResult run()
      {
        ida_.initializeSimulation(0.0, false);
        return runSegments(std::span<const SystemEvent>(study_.events.data(), study_.events.size()), 0.0, study_.tmax);
      }

      StudyRunResult runFromCheckpoint(const Checkpoint&            checkpoint,
                                       std::span<const SystemEvent> events,
                                       double                       tmax)
      {
        try
        {
          ida_.restoreSolutionCheckpoint(checkpoint);
        }
        catch (const std::exception& ex)
        {
          StudyRunResult result;
          result.solve_status      = -1;
          result.solve_status_name = ida_.getStats().solve_return_flag_name_;
          result.pending_exception = std::current_exception();
          result.pending_what      = std::string(ex.what());
          result.stats             = AnalysisManager::Sundials::IdaStatsRecorder(options_.record_stats).report(options_.log_options);
          result.steps             = AnalysisManager::Sundials::IdaStepHistoryRecorder(options_.record_steps).report();
          result.config            = makeRunConfig(tmax);
          return result;
        }

        return runSegments(events, checkpoint.time, tmax);
      }

      Checkpoint saveCheckpoint(double time) const
      {
        return ida_.saveSolutionCheckpoint(time);
      }

      void setLogger(std::optional<AnalysisManager::Sundials::IdaLogOptions> log_options)
      {
        options_.log_options = log_options;
        ida_.setLogger(std::move(log_options));
      }

      SystemModel<ScalarT, IdxT>& system()
      {
        return sys_;
      }

    private:
      using IdaT          = AnalysisManager::Sundials::Ida<ScalarT, IdxT>;
      using IdaStats      = AnalysisManager::Sundials::IdaStats;
      using IdaRunConfig  = AnalysisManager::Sundials::IdaRunConfig;
      using StatsRecorder = AnalysisManager::Sundials::IdaStatsRecorder;
      using StepsRecorder = AnalysisManager::Sundials::IdaStepHistoryRecorder;
      using SegmentKind   = AnalysisManager::Sundials::SegmentKind;
      using EventType     = SystemEvent::Type;
      using Clock         = std::chrono::steady_clock;

      void applySolverOptions()
      {
        if (study_.ida_max_order.has_value())
        {
          ida_.setMaxOrder(study_.ida_max_order.value());
        }
        if (study_.ida_max_steps.has_value())
        {
          ida_.setMaxNumSteps(study_.ida_max_steps.value());
        }
        if (study_.ida_max_dt.has_value())
        {
          ida_.setMaxStep(study_.ida_max_dt.value());
        }
        if (study_.rel_tol.has_value() && study_.abs_tol.has_value())
        {
          ida_.setTolerances(study_.rel_tol.value(), study_.abs_tol.value());
        }
      }

      std::optional<int> outputCountForSegment(double seg_start, double seg_end) const
      {
        if (options_.output_mode == StudyOutputMode::SegmentEndOnly)
        {
          return 1;
        }
        if (!sys_.monitoring())
        {
          return 1;
        }
        if (study_.dt == 0.0)
        {
          return std::nullopt;
        }
        return static_cast<int>(std::round((seg_end - seg_start) / study_.dt));
      }

      void runSegment(StudyRunResult&    result,
                      StatsRecorder&     stats_recorder,
                      StepsRecorder&     steps_recorder,
                      double             seg_start,
                      double             seg_end,
                      std::optional<int> output_count)
      {
        const int requested_output_count = output_count.value_or(0);
        stats_recorder.beginSegment(ida_);
        steps_recorder.beginSegment(ida_, seg_start, seg_end, requested_output_count);
        try
        {
          if (steps_recorder.enabled())
          {
            result.solve_status = ida_.runSimulationWithStepHistory(
                seg_end,
                output_count,
                [&](const IdaStats& stats)
                { steps_recorder.recordStep(stats); });
          }
          else
          {
            result.solve_status = ida_.runSimulation(seg_end, output_count);
          }
        }
        catch (const std::exception& ex)
        {
          result.solve_status      = -1;
          result.pending_exception = std::current_exception();
          result.pending_what      = std::string(ex.what());
        }
        // Capture diagnostics even if the segment failed.
        steps_recorder.endSegment(ida_);
        stats_recorder.endSegment(ida_, seg_start, seg_end, requested_output_count);
      }

      bool reinitializeAtEvent(StudyRunResult& result, StatsRecorder& stats_recorder, double event_time)
      {
        stats_recorder.beginSegment(ida_);
        bool calc_ic_success = true;
        try
        {
          ida_.initializeSimulation(event_time, true);
        }
        catch (const std::exception& ex)
        {
          calc_ic_success          = false;
          result.solve_status      = -1;
          result.pending_exception = std::current_exception();
          result.pending_what      = std::string(ex.what());
        }
        stats_recorder.endSegment(ida_, event_time, event_time, 0, SegmentKind::InitialCondition, calc_ic_success);
        return calc_ic_success;
      }

      void applyEvent(const SystemEvent& event)
      {
        if (event.type == EventType::FAULT_ON)
        {
          sys_.getBusFault(static_cast<IdxT>(event.element_id))->setStatus(true);
        }
        else if (event.type == EventType::FAULT_OFF)
        {
          sys_.getBusFault(static_cast<IdxT>(event.element_id))->setStatus(false);
        }
      }

      StudyRunResult runSegments(std::span<const SystemEvent> events, double start_time, double tmax)
      {
        StudyRunResult result;
        const auto     clock_start = Clock::now();

        StatsRecorder stats_recorder(options_.record_stats);
        StepsRecorder steps_recorder(options_.record_steps);

        double curr_time = start_time;
        for (const auto& event : events)
        {
          auto output_count = outputCountForSegment(curr_time, event.time);
          if (event.time > curr_time && (!output_count.has_value() || *output_count > 0))
          {
            runSegment(result, stats_recorder, steps_recorder, curr_time, event.time, output_count);
          }
          if (result.solve_status != 0)
          {
            break;
          }

          applyEvent(event);

          if (!reinitializeAtEvent(result, stats_recorder, event.time))
          {
            break;
          }
          curr_time = event.time;
        }

        auto final_output_count = outputCountForSegment(curr_time, tmax);
        if (result.solve_status == 0 && tmax > curr_time
            && (!final_output_count.has_value() || *final_output_count > 0))
        {
          runSegment(result, stats_recorder, steps_recorder, curr_time, tmax, final_output_count);
        }

        sys_.stopMonitor();

        result.solve_status_name = ida_.getStats().solve_return_flag_name_;
        result.stats             = stats_recorder.report(options_.log_options);
        result.steps             = steps_recorder.report();
        result.config            = makeRunConfig(tmax);

        const auto clock_end      = Clock::now();
        result.wall_clock_seconds = std::chrono::duration<double>(clock_end - clock_start).count();

        return result;
      }

      IdaRunConfig makeRunConfig(double tmax)
      {
        IdaRunConfig config;
#ifdef GRIDKIT_ENABLE_SUNDIALS_SPARSE
        config.linear_solver_kind = "KLU";
        config.sparse_enabled     = true;
        config.jacobian_nnz       = static_cast<long int>(sys_.getCsrJacobian()->getNnz());
#else
        config.linear_solver_kind = "dense";
        config.sparse_enabled     = false;
#endif
        config.model_size       = static_cast<long int>(sys_.size());
        config.ida_max_order    = study_.ida_max_order;
        config.ida_max_steps    = study_.ida_max_steps;
        config.ida_max_dt       = study_.ida_max_dt;
        config.rel_tol          = study_.rel_tol;
        config.abs_tol          = study_.abs_tol;
        config.mu               = study_.mu;
        config.dt               = study_.dt;
        config.tmax             = tmax;
        config.sundials_version = SUNDIALS_VERSION;
        return config;
      }

      StudyData&                 study_;
      StudyRunOptions            options_;
      SystemModel<ScalarT, IdxT> sys_;
      IdaT                       ida_;
    };

    /**
     * @brief Run one dynamic study: configure IDA, apply the max-order/step
     * controls, walk the fault-event segments, and record IDA diagnostics.
     *
     * Mirrors the segment/event behaviour of the DynamicSimulation driver:
     * `dt < 0` is rejected, `dt == 0` requests solver-selected steps, each event
     * toggles its bus fault and reinitializes consistent initial conditions, and
     * the event loop stops at the first failing segment. Unlike a bare driver the
     * consistent-initial-condition solve at each event is wrapped so its effort
     * (and success) can be recorded as its own diagnostics phase.
     *
     * @tparam ScalarT Scalar data type.
     * @tparam IdxT    Index data type.
     * @param[in,out] study   Parsed study; its model data builds the system.
     * @param[in]     options Diagnostics, logger, and segment-output controls.
     * @return Solve status, diagnostics reports, configuration, wall-clock time,
     *         and any captured exception (NOT rethrown; the caller decides).
     *
     * @post The variable monitor has been stopped. Diagnostics are recorded even
     *       when a segment throws, so the caller can flush them before rethrowing.
     */
    template <class ScalarT, class IdxT>
    StudyRunResult runDynamicStudy(StudyData& study, StudyRunOptions options)
    {
      StudyRunResult result;
      if (!detail::validateStudyConfiguration(study, result))
      {
        return result;
      }

      DynamicStudyRunner<ScalarT, IdxT> runner(study, std::move(options));
      return runner.run();
    }

    /**
     * @brief Convenience overload that preserves the old boolean diagnostics call.
     */
    template <class ScalarT, class IdxT>
    StudyRunResult runDynamicStudy(StudyData&                                              study,
                                   bool                                                    record_stats,
                                   bool                                                    record_steps,
                                   std::optional<AnalysisManager::Sundials::IdaLogOptions> log_options = {},
                                   StudyOutputMode                                         output_mode =
                                       StudyOutputMode::StudyCadence)
    {
      StudyRunOptions options;
      options.record_stats = record_stats;
      options.record_steps = record_steps;
      options.log_options  = std::move(log_options);
      options.output_mode  = output_mode;

      return runDynamicStudy<ScalarT, IdxT>(study, std::move(options));
    }
  } // namespace PhasorDynamics
} // namespace GridKit
