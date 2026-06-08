#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>

namespace AnalysisManager
{
  namespace Sundials
  {
    namespace
    {
      constexpr const char* IDA_STATS_SCHEMA = "gridkit.ida_stats.v1";
      constexpr const char* IDA_STEPS_SCHEMA = "gridkit.ida_steps.v2";
      constexpr const char* IDA_STEPS_SOURCE = "actual_solve";
      constexpr const char* IDA_STEPS_DRIVER = "IDA_ONE_STEP";

      std::string logLevelName(IdaLogLevel level)
      {
        return level == IdaLogLevel::Error ? "error" : "warning";
      }

      std::string segmentKindName(SegmentKind kind)
      {
        return kind == SegmentKind::InitialCondition ? "initial_condition" : "solve";
      }

      // Counter blocks shared by the aggregate stats and the per-step deltas.
      // finite_difference_residual_evals is IDAGetNumLinResEvals: residual calls
      // from internal finite-difference Jacobian/J*v approximations only
      // (approximately 0 when an analytic Jacobian is supplied).
      nlohmann::json idaCounterJson(const IdaStats& stats)
      {
        return nlohmann::json{
            {"integrator",
             {{"steps", stats.num_steps_},
              {"residual_evals", stats.num_residual_evals_},
              {"linear_solver_setups", stats.num_linear_solver_setups_},
              {"error_test_failures", stats.num_error_test_fails_},
              {"backtrack_operations", stats.num_backtrack_operations_}}},
            {"nonlinear_solver",
             {{"iterations", stats.num_nonlinear_iters_},
              {"convergence_failures", stats.num_nonlinear_convergence_fails_},
              {"step_solve_failures", stats.num_nonlinear_step_fails_}}},
            {"linear_solver",
             {{"jacobian_evals", stats.num_jacobian_evals_},
              {"iterations", stats.num_linear_iters_},
              {"convergence_failures", stats.num_linear_convergence_fails_},
              {"finite_difference_residual_evals", stats.num_linear_residual_evals_}}}};
      }

      nlohmann::json idaFinalStateJson(const IdaStats& stats)
      {
        return nlohmann::json{
            {"last_order", stats.last_order_},
            {"current_order", stats.current_order_},
            {"actual_initial_step", stats.actual_initial_step_},
            {"last_step", stats.last_step_},
            {"current_step", stats.current_step_},
            {"current_time", stats.current_time_},
            {"current_cj", stats.current_cj_},
            {"tol_scale_factor", stats.tol_scale_factor_}};
      }

      nlohmann::json idaStatsJson(const IdaStats& stats)
      {
        auto json        = idaCounterJson(stats);
        json["sundials"] = {
            {"version", stats.sundials_version_},
            {"logging_level", stats.sundials_logging_level_}};

        // jacobian_eval_time is IDAGetJacTime: the simulation time of the last
        // Jacobian evaluation, not wall-clock compute time.
        auto& linear_solver                    = json["linear_solver"];
        linear_solver["last_jacobian_step"]    = stats.last_jacobian_step_;
        linear_solver["jacobian_eval_time"]    = stats.jacobian_eval_time_;
        linear_solver["jacobian_cj"]           = stats.jacobian_cj_;
        linear_solver["preconditioner_evals"]  = stats.num_preconditioner_evals_;
        linear_solver["preconditioner_solves"] = stats.num_preconditioner_solves_;
        linear_solver["jtimes_setup_evals"]    = stats.num_jtimes_setup_evals_;
        linear_solver["jtimes_evals"]          = stats.num_jtimes_evals_;
        linear_solver["last_flag"]             = stats.last_linear_flag_;
        linear_solver["last_flag_name"]        = stats.last_linear_flag_name_;

        json["final_state"] = idaFinalStateJson(stats);
        return json;
      }

      nlohmann::json idaStepCounterDeltaJson(const IdaStats& stats)
      {
        return idaCounterJson(stats);
      }

      nlohmann::json idaRunConfigJson(const IdaRunConfig& config)
      {
        return nlohmann::json{
            {"linear_solver_kind", config.linear_solver_kind},
            {"sparse_enabled", config.sparse_enabled},
            {"model_size", config.model_size},
            {"jacobian_nnz", config.jacobian_nnz.has_value() ? nlohmann::json(*config.jacobian_nnz) : nlohmann::json()},
            {"ida_max_order", config.ida_max_order.has_value() ? nlohmann::json(*config.ida_max_order) : nlohmann::json()},
            {"ida_max_dt", config.ida_max_dt.has_value() ? nlohmann::json(*config.ida_max_dt) : nlohmann::json()},
            {"rel_tol", config.rel_tol.has_value() ? nlohmann::json(*config.rel_tol) : nlohmann::json()},
            {"abs_tol", config.abs_tol.has_value() ? nlohmann::json(*config.abs_tol) : nlohmann::json()},
            {"mu", config.mu},
            {"dt", config.dt},
            {"tmax", config.tmax},
            {"sundials_version", config.sundials_version}};
      }

      void addOptionalSections(nlohmann::json& json, const IdaDiagnosticsOutput& output)
      {
        if (output.config.has_value())
        {
          json["config"] = idaRunConfigJson(*output.config);
        }
        if (output.wall_clock_seconds.has_value())
        {
          json["timing"] = {{"wall_clock_seconds", *output.wall_clock_seconds}};
          if (output.timing_scope.has_value())
          {
            json["timing"]["scope"] = *output.timing_scope;
          }
        }
      }

      nlohmann::json idaStepSampleJson(const IdaStepSample& sample)
      {
        return nlohmann::json{
            {"segment_index", sample.segment_index},
            {"segment_step_index", sample.segment_step_index},
            {"global_step", sample.global_step},
            {"step_start_time", sample.step_start_time},
            {"step_end_time", sample.step_end_time},
            {"last_step", sample.last_step},
            {"current_step", sample.current_step},
            {"last_order", sample.last_order},
            {"current_order", sample.current_order},
            {"current_cj", sample.current_cj},
            {"counter_delta", idaStepCounterDeltaJson(sample.counter_delta)}};
      }

      nlohmann::json idaStepHistorySegmentJson(const IdaStepHistorySegment& segment)
      {
        nlohmann::json json{
            {"segment_index", segment.segment_index},
            {"start_time", segment.start_time},
            {"end_time", segment.end_time},
            {"output_steps", segment.output_steps},
            {"accepted_steps", segment.steps.size()},
            {"reached_time", segment.final_stats.current_time_},
            {"final_state", idaFinalStateJson(segment.final_stats)},
            {"steps", nlohmann::json::array()}};

        for (const auto& sample : segment.steps)
        {
          json["steps"].push_back(idaStepSampleJson(sample));
        }
        return json;
      }

      nlohmann::json idaStatsSegmentJson(const IdaStatsSegment& segment)
      {
        auto json = idaStatsJson(segment.stats);
        json.erase("sundials");
        json["phase"]        = segmentKindName(segment.kind);
        json["start_time"]   = segment.start_time;
        json["end_time"]     = segment.end_time;
        json["output_steps"] = segment.output_steps;
        if (segment.kind == SegmentKind::InitialCondition)
        {
          if (segment.calc_ic_success.has_value())
          {
            json["calc_ic_success"] = *segment.calc_ic_success;
          }
        }
        else
        {
          json["solve_status"]      = segment.stats.solve_return_flag_;
          json["solve_status_name"] = segment.stats.solve_return_flag_name_;
        }
        return json;
      }
    } // namespace

    IdaStatsRecorder::IdaStatsRecorder(bool enabled)
      : enabled_(enabled)
    {
    }

    bool IdaStatsRecorder::enabled() const
    {
      return enabled_;
    }

    IdaStepHistoryRecorder::IdaStepHistoryRecorder(bool enabled)
      : enabled_(enabled)
    {
    }

    bool IdaStepHistoryRecorder::enabled() const
    {
      return enabled_;
    }

    void IdaStepHistoryRecorder::recordStep(const IdaStats& stats)
    {
      if (!enabled_)
      {
        return;
      }
      if (!current_segment_.has_value() || !previous_step_stats_.has_value())
      {
        throw std::logic_error("IDA step-history sample recorded before a segment was started");
      }

      auto counter_delta = idaStatsDelta(stats, *previous_step_stats_);
      if (counter_delta.num_steps_ <= 0)
      {
        previous_step_stats_ = stats;
        return;
      }

      accepted_steps_ += static_cast<std::size_t>(counter_delta.num_steps_);

      IdaStepSample sample;
      sample.segment_index      = current_segment_->segment_index;
      sample.segment_step_index = current_segment_->steps.size() + 1;
      sample.global_step        = accepted_steps_;
      sample.step_start_time    = stats.current_time_ - stats.last_step_;
      sample.step_end_time      = stats.current_time_;
      sample.last_step          = stats.last_step_;
      sample.current_step       = stats.current_step_;
      sample.last_order         = stats.last_order_;
      sample.current_order      = stats.current_order_;
      sample.current_cj         = stats.current_cj_;
      sample.counter_delta      = std::move(counter_delta);

      current_segment_->steps.push_back(std::move(sample));
      previous_step_stats_ = stats;
    }

    IdaStepHistoryReport IdaStepHistoryRecorder::report() const
    {
      return {segments_};
    }

    void IdaStatsRecorder::recordSegment(const IdaStats&     start_stats,
                                         const IdaStats&     end_stats,
                                         double              start_time,
                                         double              end_time,
                                         int                 output_steps,
                                         SegmentKind         kind,
                                         std::optional<bool> calc_ic_success)
    {
      auto stats  = idaStatsDelta(end_stats, start_stats);
      summary_   += stats;
      segments_.push_back({start_time, end_time, output_steps, kind, calc_ic_success, std::move(stats)});
    }

    IdaStatsReport IdaStatsRecorder::report(std::optional<IdaLogOptions> log) const
    {
      return {summary_, segments_, std::move(log)};
    }

    IdaStats idaStatsDelta(const IdaStats& end_stats, const IdaStats& start_stats)
    {
      IdaStats delta = end_stats;

      delta.num_steps_                       = end_stats.num_steps_ - start_stats.num_steps_;
      delta.num_residual_evals_              = end_stats.num_residual_evals_ - start_stats.num_residual_evals_;
      delta.num_linear_solver_setups_        = end_stats.num_linear_solver_setups_ - start_stats.num_linear_solver_setups_;
      delta.num_error_test_fails_            = end_stats.num_error_test_fails_ - start_stats.num_error_test_fails_;
      delta.num_backtrack_operations_        = end_stats.num_backtrack_operations_ - start_stats.num_backtrack_operations_;
      delta.num_nonlinear_iters_             = end_stats.num_nonlinear_iters_ - start_stats.num_nonlinear_iters_;
      delta.num_nonlinear_convergence_fails_ = end_stats.num_nonlinear_convergence_fails_
                                               - start_stats.num_nonlinear_convergence_fails_;
      delta.num_nonlinear_step_fails_     = end_stats.num_nonlinear_step_fails_ - start_stats.num_nonlinear_step_fails_;
      delta.num_jacobian_evals_           = end_stats.num_jacobian_evals_ - start_stats.num_jacobian_evals_;
      delta.num_linear_iters_             = end_stats.num_linear_iters_ - start_stats.num_linear_iters_;
      delta.num_linear_convergence_fails_ = end_stats.num_linear_convergence_fails_
                                            - start_stats.num_linear_convergence_fails_;
      delta.num_linear_residual_evals_ = end_stats.num_linear_residual_evals_ - start_stats.num_linear_residual_evals_;
      delta.num_preconditioner_evals_  = end_stats.num_preconditioner_evals_ - start_stats.num_preconditioner_evals_;
      delta.num_preconditioner_solves_ = end_stats.num_preconditioner_solves_ - start_stats.num_preconditioner_solves_;
      delta.num_jtimes_setup_evals_    = end_stats.num_jtimes_setup_evals_ - start_stats.num_jtimes_setup_evals_;
      delta.num_jtimes_evals_          = end_stats.num_jtimes_evals_ - start_stats.num_jtimes_evals_;

      return delta;
    }

    IdaStatsReport combineIdaStatsReports(const IdaStatsReport&        prefix,
                                          const IdaStatsReport&        suffix,
                                          std::optional<IdaLogOptions> log)
    {
      IdaStatsReport combined;
      combined.summary   = prefix.summary;
      combined.summary  += suffix.summary;
      combined.segments  = prefix.segments;
      combined.segments.insert(combined.segments.end(), suffix.segments.begin(), suffix.segments.end());
      combined.log = std::move(log);
      return combined;
    }

    void writeIdaStatsJson(const IdaStatsReport& report, const IdaDiagnosticsOutput& output)
    {
      std::ofstream stream(output.file);
      if (!stream)
      {
        throw std::runtime_error("failed to open IDA stats output file '" + output.file.string() + "'");
      }

      auto json      = idaStatsJson(report.summary);
      json["schema"] = IDA_STATS_SCHEMA;
      addOptionalSections(json, output);
      json["segment_count"] = report.segments.size();
      json["segments"]      = nlohmann::json::array();
      for (const auto& segment : report.segments)
      {
        json["segments"].push_back(idaStatsSegmentJson(segment));
      }

      const auto& log = output.log.has_value() ? output.log : report.log;
      if (log.has_value())
      {
        json["log"] = {
            {"file", log->file.string()},
            {"level", logLevelName(log->level)}};
      }

      stream << json.dump(2) << '\n';
    }

    void writeIdaStepHistoryJson(const IdaStepHistoryReport& report, const IdaDiagnosticsOutput& output)
    {
      std::ofstream stream(output.file);
      if (!stream)
      {
        throw std::runtime_error("failed to open IDA step-history output file '" + output.file.string() + "'");
      }

      nlohmann::json json{
          {"schema", IDA_STEPS_SCHEMA},
          {"source", IDA_STEPS_SOURCE},
          {"driver", IDA_STEPS_DRIVER},
          {"stop_time_enforced", true},
          {"segment_count", report.segments.size()},
          {"segments", nlohmann::json::array()}};

      addOptionalSections(json, output);

      if (!report.segments.empty())
      {
        const auto& stats = report.segments.back().final_stats;
        json["sundials"]  = {
            {"version", stats.sundials_version_},
            {"logging_level", stats.sundials_logging_level_}};
      }

      for (const auto& segment : report.segments)
      {
        json["segments"].push_back(idaStepHistorySegmentJson(segment));
      }

      stream << json.dump(2) << '\n';
    }
  } // namespace Sundials
} // namespace AnalysisManager
