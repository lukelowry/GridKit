#include <optional>
#include <stdexcept>

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

      NullEvaluator(RealT rel_tol = 1.0e-6, RealT abs_tol = 1.0e-8)
        : rel_tol_(rel_tol),
          abs_tol_(abs_tol)
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
        rel_tol = rel_tol_;
        abs_tol = abs_tol_;
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

      RealT rel_tol_;
      RealT abs_tol_;
    };

    template <class ScalarT, typename IdxT>
    class DecayEvaluator : public NullEvaluator<ScalarT, IdxT>
    {
    public:
      using RealT = typename NullEvaluator<ScalarT, IdxT>::RealT;

      DecayEvaluator(RealT rel_tol = 1.0e-8, RealT abs_tol = 1.0e-10)
        : NullEvaluator<ScalarT, IdxT>(rel_tol, abs_tol)
      {
      }

      int initialize() override
      {
        this->y_  = {1};
        this->yp_ = {-1};

        this->tag_ = {true};

        this->f_ = {0};
        this->g_ = {0};
        return 0;
      }

      int tagDifferentiable() override
      {
        this->tag_ = {true};
        return 0;
      }

      int evaluateResidual() override
      {
        this->f_[0] = this->yp_[0] + this->y_[0];
        return 0;
      }
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
        ida.setMaxOrder(2);

        unsigned observed_steps = 0;
        auto     output_cb      = [&]([[maybe_unused]] double t)
        {
          observed_steps++;
        };

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(1.0, n_steps, output_cb);

        success *= (observed_steps == n_steps);

        Model::NullEvaluator<ScalarT, IdxT> solver_step_model;
        Ida<double, size_t>                 solver_step_ida(&solver_step_model);
        solver_step_ida.configureSimulation();

        unsigned observed_solver_steps = 0;
        auto     solver_step_cb        = [&]([[maybe_unused]] double t)
        {
          observed_solver_steps++;
        };

        solver_step_ida.initializeSimulation(0.0, false);
        solver_step_ida.runSimulation(1.0, std::nullopt, solver_step_cb);

        const auto solver_step_stats  = solver_step_ida.getStats();
        success                      *= (observed_solver_steps > 0);
        success                      *= (observed_solver_steps == static_cast<unsigned>(solver_step_stats.num_steps_));

        Model::DecayEvaluator<ScalarT, IdxT> checkpoint_model;
        Ida<double, size_t>                  checkpoint_ida(&checkpoint_model);
        checkpoint_ida.configureSimulation();
        checkpoint_ida.initializeSimulation(0.0, false);
        checkpoint_ida.runSimulation(0.5, 1);
        const auto checkpoint = checkpoint_ida.saveSolutionCheckpoint(0.5);
        checkpoint_ida.runSimulation(1.0, 1);
        const auto first_final_y = checkpoint_model.y()[0];
        checkpoint_ida.restoreSolutionCheckpoint(checkpoint);
        success *= isEqual(checkpoint_model.y()[0], checkpoint.y[0], 1.0e-12);
        success *= isEqual(checkpoint_model.yp()[0], checkpoint.yp[0], 1.0e-12);
        checkpoint_ida.runSimulation(1.0, 1);
        success *= isEqual(checkpoint_model.y()[0], first_final_y, 1.0e-8);

        Model::NullEvaluator<ScalarT, IdxT> invalid_model(0.0, 1.0e-8);
        Ida<double, size_t>                 invalid_ida(&invalid_model);
        bool                                invalid_tolerance_rejected = false;
        try
        {
          invalid_ida.configureSimulation();
        }
        catch (const std::invalid_argument&)
        {
          invalid_tolerance_rejected = true;
        }
        success *= invalid_tolerance_rejected;

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
