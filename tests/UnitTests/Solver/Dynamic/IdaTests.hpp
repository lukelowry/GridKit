#include <algorithm>
#include <cmath>

#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Model/LogEvaluator.hpp>
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
        rel_tol = 1.0e-7;
        abs_tol = 1.0e-9;
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
    };

    template <class ScalarT, typename IdxT>
    class AlgebraicRootEvaluator : public NullEvaluator<ScalarT, IdxT>
    {
    public:
      using RealT = typename NullEvaluator<ScalarT, IdxT>::RealT;

      int initialize() override
      {
        NullEvaluator<ScalarT, IdxT>::initialize();
        this->y_[0] = 1.0;
        time_       = 1.0;
        return 0;
      }

      int evaluateResidual() override
      {
        this->f_[0] = this->y_[0] * this->y_[0] - static_cast<ScalarT>(time_);
        return 0;
      }

      void updateTime(RealT t, RealT) override
      {
        time_ = t;
      }

    private:
      RealT time_{1.0};
    };

    template <class ScalarT, typename IdxT>
    class DifferentialRampEvaluator : public NullEvaluator<ScalarT, IdxT>
    {
    public:
      using RealT = typename NullEvaluator<ScalarT, IdxT>::RealT;

      int initialize() override
      {
        NullEvaluator<ScalarT, IdxT>::initialize();
        this->y_[0]   = static_cast<ScalarT>(coordinate_);
        this->yp_[0]  = 1.0;
        this->tag_[0] = true;
        return 0;
      }

      int evaluateResidual() override
      {
        this->f_[0] = this->yp_[0] - 1.0;
        return 0;
      }

      void updateTime(RealT coordinate, RealT alpha) override
      {
        coordinate_ = coordinate;
        alpha_      = alpha;
      }

      RealT coordinate() const
      {
        return coordinate_;
      }

      RealT alpha() const
      {
        return alpha_;
      }

    private:
      RealT coordinate_{1.0};
      RealT alpha_{0.0};
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

        Ida<ScalarT, IdxT> ida(&model);
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

      TestOutcome algebraic_error_control()
      {
        Model::AlgebraicRootEvaluator<ScalarT, IdxT> model;

        Ida<ScalarT, IdxT> ida(&model);
        ida.configureSimulation();

        ScalarT max_error = 0.0;
        auto    output_cb = [&](ScalarT t)
        {
          const ScalarT expected = std::sqrt(t);
          max_error              = std::max(max_error, std::abs(model.y()[0] - expected));
        };

        ida.initializeSimulation(1.0, false);
        ida.runSimulation(100.0, 20, output_cb);

        TestStatus success  = true;
        success            *= (max_error < 1.0e-3);

        return success.report(__func__);
      }

      TestOutcome log_evaluator_algebraic()
      {
        Model::AlgebraicRootEvaluator<ScalarT, IdxT> model;
        Model::LogEvaluator<ScalarT, IdxT>           log_model(model, 1.0);

        log_model.allocate();

        Ida<ScalarT, IdxT> ida(&log_model);
        ida.configureSimulation();

        ScalarT max_error = 0.0;
        auto    output_cb = [&](ScalarT s)
        {
          const ScalarT expected = std::sqrt(std::exp(s));
          max_error              = std::max(max_error, std::abs(log_model.y()[0] - expected));
        };

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(std::log(100.0), 20, output_cb);

        TestStatus success  = true;
        success            *= (max_error < 1.0e-3);

        return success.report(__func__);
      }

      TestOutcome log_evaluator_derivative_scaling()
      {
        Model::DifferentialRampEvaluator<ScalarT, IdxT> model;
        Model::LogEvaluator<ScalarT, IdxT>              log_model(model, 1.0);

        log_model.allocate();

        Ida<ScalarT, IdxT> ida(&log_model);
        ida.configureSimulation();
        ida.initializeSimulation(0.0, false);
        ida.runSimulation(std::log(10.0), 20);

        TestStatus success  = true;
        success            *= isEqual(log_model.y()[0], static_cast<ScalarT>(10.0), static_cast<ScalarT>(1.0e-5));

        return success.report(__func__);
      }

      TestOutcome log_evaluator_alpha_scaling()
      {
        Model::DifferentialRampEvaluator<ScalarT, IdxT> model;
        Model::LogEvaluator<ScalarT, IdxT>              log_model(model, 1.0);

        log_model.allocate();
        log_model.initialize();
        log_model.updateTime(std::log(10.0), 30.0);

        TestStatus success  = true;
        success            *= isEqual(model.coordinate(), static_cast<ScalarT>(10.0), static_cast<ScalarT>(1.0e-12));
        success            *= isEqual(model.alpha(), static_cast<ScalarT>(3.0), static_cast<ScalarT>(1.0e-12));

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
