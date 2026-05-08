#include <limits>

#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
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

        tag_ = {false};

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

      void setTolerances(RealT& rel_tol, RealT& abs_tol) const override
      {
        rel_tol = 1e-8;
        abs_tol = 1e-10;
      }

      void setMaxSteps(IdxT& msa) const override
      {
        msa = 2000;
      }

      void setMaxStepSize(RealT& hmax) const override
      {
        hmax = max_step_size_;
      }

      void setRequestedMaxStepSize(RealT max_step_size)
      {
        max_step_size_ = max_step_size;
      }

      int stepAccepted(RealT t) override
      {
        accepted_times_.push_back(t);
        return 0;
      }

      const std::vector<RealT>& acceptedTimes() const
      {
        return accepted_times_;
      }

      int tagDifferentiable() override
      {
        return 0;
      }

      int evaluateResidual() override
      {
        f_ = y_;
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

      RealT              max_step_size_{std::numeric_limits<RealT>::infinity()};
      std::vector<RealT> accepted_times_;
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

        Model::NullEvaluator<ScalarT, IdxT> max_step_model;
        max_step_model.setRequestedMaxStepSize(0.05);

        Ida<double, size_t> max_step_ida(&max_step_model);
        max_step_ida.configureSimulation();

        unsigned max_step_observed_outputs = 0;
        auto     max_step_output_cb        = [&]([[maybe_unused]] double t)
        {
          max_step_observed_outputs++;
        };

        max_step_ida.initializeSimulation(0.0, false);
        max_step_ida.runSimulation(1.0, 10, max_step_output_cb);

        const auto& accepted_times  = max_step_model.acceptedTimes();
        success                    *= (max_step_observed_outputs == 10);
        success                    *= (accepted_times.size() > 10);
        success                    *= (!accepted_times.empty());

        constexpr double max_step_tolerance = 1.0e-10;
        for (size_t i = 1; i < accepted_times.size(); ++i)
        {
          const auto spacing  = accepted_times[i] - accepted_times[i - 1];
          success            *= (spacing >= -max_step_tolerance);
          success            *= (spacing <= 0.05 + max_step_tolerance);
        }

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
