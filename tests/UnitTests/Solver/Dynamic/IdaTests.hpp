#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Solver/Dynamic/IdaDiagnostics.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

using AnalysisManager::Sundials::Ida;

namespace GridKit
{
  namespace Model
  {
    template <class ScalarT, typename IdxT>
    class NullEvaluator : public Model::Evaluator<ScalarT, IdxT>
    {
    public:
      using RealT = typename Model::Evaluator<ScalarT, IdxT>::RealT;

      NullEvaluator()
      {
      }

      int allocate() override
      {
        return 0;
      }

      int initialize() override
      {
        y_  = {0};
        yp_ = {0};

        tag_ = {true};

        f_ = {0};
        g_ = {0};
        return 0;
      }

      IdxT size() override
      {
        return 1;
      }

      IdxT nnz() override
      {
        return 0;
      }

      bool hasJacobian() override
      {
        return false;
      }

      IdxT sizeQuadrature() override
      {
        return 0;
      }

      IdxT sizeParams() override
      {
        return 0;
      }

      void setTolerances([[maybe_unused]] RealT& rel_tol, [[maybe_unused]] RealT& abs_tol) const override
      {
        rel_tol = 1.0e-8;
        abs_tol = 1.0e-8;
      }

      void setMaxSteps(IdxT& msa) const override
      {
        msa = 2000;
      }

      int tagDifferentiable() override
      {
        return 0;
      }

      int evaluateResidual() override
      {
        f_ = yp_;
        return 0;
      }

      int evaluateJacobian() override
      {
        return 0;
      }

      int evaluateIntegrand() override
      {
        return 0;
      }

      int initializeAdjoint() override
      {
        return 0;
      }

      int evaluateAdjointResidual() override
      {
        return 0;
      }

      int evaluateAdjointIntegrand() override
      {
        return 0;
      }

      void updateTime([[maybe_unused]] RealT t, [[maybe_unused]] RealT a) override
      {
      }

      std::vector<ScalarT>& y() override
      {
        return y_;
      }

      const std::vector<ScalarT>& y() const override
      {
        return y_;
      }

      std::vector<ScalarT>& yp() override
      {
        return yp_;
      }

      const std::vector<ScalarT>& yp() const override
      {
        return yp_;
      }

      std::vector<bool>& tag() override
      {
        return tag_;
      }

      const std::vector<bool>& tag() const override
      {
        return tag_;
      }

      std::vector<ScalarT>& yB() override
      {
        return yB_;
      }

      const std::vector<ScalarT>& yB() const override
      {
        return yB_;
      }

      std::vector<ScalarT>& ypB() override
      {
        return ypB_;
      }

      const std::vector<ScalarT>& ypB() const override
      {
        return ypB_;
      }

      std::vector<ScalarT>& param() override
      {
        return param_;
      }

      const std::vector<ScalarT>& param() const override
      {
        return param_;
      }

      std::vector<ScalarT>& param_up() override
      {
        return param_up_;
      }

      const std::vector<ScalarT>& param_up() const override
      {
        return param_up_;
      }

      std::vector<ScalarT>& param_lo() override
      {
        return param_lo_;
      }

      const std::vector<ScalarT>& param_lo() const override
      {
        return param_lo_;
      }

      std::vector<ScalarT>& getResidual() override
      {
        return f_;
      }

      const std::vector<ScalarT>& getResidual() const override
      {
        return f_;
      }

      GridKit::LinearAlgebra::COO_Matrix<ScalarT, IdxT>& getJacobian() override
      {
        return jac_;
      }

      const GridKit::LinearAlgebra::COO_Matrix<ScalarT, IdxT>& getJacobian() const override
      {
        return jac_;
      }

      std::vector<ScalarT>& getIntegrand() override
      {
        return g_;
      }

      const std::vector<ScalarT>& getIntegrand() const override
      {
        return g_;
      }

      std::vector<ScalarT>& getAdjointResidual() override
      {
        return fB_;
      }

      const std::vector<ScalarT>& getAdjointResidual() const override
      {
        return fB_;
      }

      std::vector<ScalarT>& getAdjointIntegrand() override
      {
        return gB_;
      }

      const std::vector<ScalarT>& getAdjointIntegrand() const override
      {
        return gB_;
      }

      IdxT getIDcomponent()
      {
        return 0;
      }

    protected:
      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<bool>    tag_;
      std::vector<ScalarT> f_;
      std::vector<ScalarT> g_;

      std::vector<ScalarT> yB_;
      std::vector<ScalarT> ypB_;
      std::vector<ScalarT> fB_;
      std::vector<ScalarT> gB_;

      GridKit::LinearAlgebra::COO_Matrix<ScalarT, IdxT> jac_;

      std::vector<ScalarT> param_;
      std::vector<ScalarT> param_up_;
      std::vector<ScalarT> param_lo_;
    };
  } // namespace Model

  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class IdaTests
    {
    public:
      TestOutcome test()
      {
        const unsigned n_steps = 100;
        TestStatus     success = true;

        Model::NullEvaluator<ScalarT, IdxT> model;

        Ida<double, size_t> ida(&model);
        ida.configureSimulation();

        unsigned observed_steps = 0;
        auto     output_cb      = [&]([[maybe_unused]] double t)
        {
          observed_steps++;
        };

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(1.0, n_steps, output_cb);

        success *= (observed_steps == n_steps);

        return success.report(__func__);
      }

      TestOutcome diagnosticsRecorder()
      {
        using AnalysisManager::Sundials::IdaStatsRecorder;

        TestStatus success = true;

        auto start                = makeStats();
        start.num_steps_          = 5;
        start.num_residual_evals_ = 10;
        start.num_jacobian_evals_ = 2;

        auto end                = makeStats();
        end.num_steps_          = 12;
        end.num_residual_evals_ = 18;
        end.num_jacobian_evals_ = 5;
        end.current_time_       = 2.0;

        FakeIda          fake({start, end});
        IdaStatsRecorder recorder(true);
        recorder.beginSegment(fake);
        recorder.endSegment(fake, 1.0, 2.0, 10);

        const auto report  = recorder.report();
        success           *= (fake.callCount() == 2);
        success           *= (report.segments.size() == 1u);
        success           *= (report.summary.num_steps_ == 7);
        success           *= (report.summary.num_residual_evals_ == 8);
        success           *= (report.summary.num_jacobian_evals_ == 3);
        success           *= isEqual(report.summary.current_time_, 2.0);
        if (report.segments.size() == 1u)
        {
          success *= isEqual(report.segments[0].start_time, 1.0);
          success *= isEqual(report.segments[0].end_time, 2.0);
          success *= (report.segments[0].output_steps == 10);
          success *= (report.segments[0].stats.num_steps_ == 7);
        }

        FakeIda          disabled_fake({makeStats()});
        IdaStatsRecorder disabled(false);
        disabled.beginSegment(disabled_fake);
        disabled.endSegment(disabled_fake, 0.0, 1.0, 1);
        success *= (disabled_fake.callCount() == 0);
        success *= disabled.report().segments.empty();

        return success.report(__func__);
      }

      TestOutcome diagnosticsJsonOutput()
      {
        using AnalysisManager::Sundials::IdaDiagnosticsOutput;
        using AnalysisManager::Sundials::IdaLogLevel;
        using AnalysisManager::Sundials::IdaLogOptions;
        using AnalysisManager::Sundials::IdaStatsReport;
        using AnalysisManager::Sundials::IdaStatsSegment;
        using AnalysisManager::Sundials::writeIdaStatsJson;

        TestStatus success = true;

        const auto dir = std::filesystem::path("IdaDiagnosticsStatsTest");
        std::filesystem::remove_all(dir);
        std::filesystem::create_directory(dir);

        auto summary                = makeStats();
        summary.num_steps_          = 7;
        summary.num_residual_evals_ = 8;
        summary.num_jacobian_evals_ = 11;
        summary.current_time_       = 0.06;

        auto segment_stats          = makeStats();
        segment_stats.num_steps_    = 4;
        segment_stats.current_time_ = 0.01;

        IdaLogOptions  log{dir / "stats.log", IdaLogLevel::Warning};
        IdaStatsReport report{summary, {IdaStatsSegment{0.0, 0.01, 100, segment_stats}}, log};
        writeIdaStatsJson(report, IdaDiagnosticsOutput{dir / "stats.json", log});

        std::ifstream  json_in(dir / "stats.json");
        nlohmann::json parsed;
        json_in >> parsed;

        success *= (parsed["sundials"]["version"].get<std::string>() == "7.mock");
        success *= (parsed["segment_count"].get<int>() == 1);
        success *= (parsed["integrator"]["steps"].get<int>() == 7);
        success *= (parsed["integrator"]["residual_evals"].get<int>() == 8);
        success *= (parsed["linear_solver"]["jacobian_evals"].get<int>() == 11);
        success *= isEqual(parsed["final_state"]["current_time"].get<double>(), 0.06);
        success *= (parsed["log"]["file"].get<std::string>() == (dir / "stats.log").string());
        success *= (parsed["log"]["level"].get<std::string>() == "warning");
        success *= (parsed["segments"].size() == 1u);
        if (parsed["segments"].size() == 1u)
        {
          success *= isEqual(parsed["segments"][0]["start_time"].get<double>(), 0.0);
          success *= isEqual(parsed["segments"][0]["end_time"].get<double>(), 0.01);
          success *= (parsed["segments"][0]["output_steps"].get<int>() == 100);
          success *= (parsed["segments"][0]["integrator"]["steps"].get<int>() == 4);
          success *= !parsed["segments"][0].contains("sundials");
          success *= !parsed["segments"][0].contains("segment_count");
        }

        std::filesystem::remove_all(dir);
        return success.report(__func__);
      }

    private:
      using IdaStats = AnalysisManager::Sundials::IdaStats;

      class FakeIda
      {
      public:
        explicit FakeIda(std::vector<IdaStats> stats)
          : stats_(std::move(stats))
        {
        }

        IdaStats getStats() const
        {
          const auto index = calls_ < stats_.size() ? calls_ : stats_.size() - 1u;
          ++calls_;
          return stats_[index];
        }

        std::size_t callCount() const
        {
          return calls_;
        }

      private:
        std::vector<IdaStats> stats_;
        mutable std::size_t   calls_{0};
      };

      static IdaStats makeStats()
      {
        IdaStats stats;
        stats.sundials_version_                = "7.mock";
        stats.sundials_logging_level_          = 4;
        stats.num_linear_solver_setups_        = 9;
        stats.num_error_test_fails_            = 1;
        stats.num_backtrack_operations_        = 4;
        stats.num_nonlinear_iters_             = 10;
        stats.num_nonlinear_convergence_fails_ = 2;
        stats.num_nonlinear_step_fails_        = 5;
        stats.num_jacobian_eval_steps_         = 6;
        stats.num_linear_iters_                = 12;
        stats.num_linear_convergence_fails_    = 3;
        stats.num_linear_residual_evals_       = 13;
        stats.num_preconditioner_evals_        = 14;
        stats.num_preconditioner_solves_       = 15;
        stats.num_jtimes_setup_evals_          = 16;
        stats.num_jtimes_evals_                = 17;
        stats.last_linear_flag_                = 0;
        stats.last_linear_flag_name_           = "SUN_SUCCESS";
        stats.last_order_                      = 2;
        stats.current_order_                   = 3;
        stats.actual_initial_step_             = 1.0e-6;
        stats.last_step_                       = 2.0e-6;
        stats.current_step_                    = 3.0e-6;
        stats.current_cj_                      = 4.0;
        stats.jacobian_time_                   = 0.05;
        stats.jacobian_cj_                     = 5.0;
        return stats;
      }
    };
  } // namespace Testing
} // namespace GridKit
