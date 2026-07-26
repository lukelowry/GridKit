#include <cmath>
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
      using RealT   = typename Model::Evaluator<ScalarT, IdxT>::RealT;
      using VectorT = typename Model::Evaluator<ScalarT, IdxT>::VectorT;

      NullEvaluator()
      {
      }

      int allocate() override
      {
        if (!allocated_)
        {
          allocateVectors(size());
          allocated_ = true;
        }
        return 0;
      }

      int initialize() override
      {
        if (!allocated_)
        {
          allocate();
        }

        auto* y       = y_.getData();
        auto* yp      = yp_.getData();
        auto* abs_tol = abs_tol_.getData();
        auto* f       = f_.getData();

        y[0]       = 0.0;
        yp[0]      = 0.0;
        tag_       = {false};
        abs_tol[0] = 0.0;
        f[0]       = 0.0;
        y_.setDataUpdated();
        yp_.setDataUpdated();
        abs_tol_.setDataUpdated();
        f_.setDataUpdated();
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

      int tagDifferentiable() override
      {
        return 0;
      }

      int setAbsoluteTolerance(RealT rel_tol) override
      {
        abs_tol_.setToConst(static_cast<ScalarT>(rel_tol));
        return 0;
      }

      int evaluateResidual() override
      {
        auto*       f = f_.getData();
        const auto* y = y_.getData();
        f[0]          = y[0];
        f_.setDataUpdated();
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

      VectorT& y() override
      {
        return y_;
      }

      const VectorT& y() const override
      {
        return y_;
      }

      VectorT& yp() override
      {
        return yp_;
      }

      const VectorT& yp() const override
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

      VectorT& absoluteTolerance() override
      {
        return abs_tol_;
      }

      const VectorT& absoluteTolerance() const override
      {
        return abs_tol_;
      }

      VectorT& yB() override
      {
        return yB_;
      }

      const VectorT& yB() const override
      {
        return yB_;
      }

      VectorT& ypB() override
      {
        return ypB_;
      }

      const VectorT& ypB() const override
      {
        return ypB_;
      }

      VectorT& param() override
      {
        return param_;
      }

      const VectorT& param() const override
      {
        return param_;
      }

      VectorT& param_up() override
      {
        return param_up_;
      }

      const VectorT& param_up() const override
      {
        return param_up_;
      }

      VectorT& param_lo() override
      {
        return param_lo_;
      }

      const VectorT& param_lo() const override
      {
        return param_lo_;
      }

      VectorT& getResidual() override
      {
        return f_;
      }

      const VectorT& getResidual() const override
      {
        return f_;
      }

      GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>* getCsrJacobian() const override
      {
        return csr_jac_;
      }

      VectorT& getIntegrand() override
      {
        return g_;
      }

      const VectorT& getIntegrand() const override
      {
        return g_;
      }

      VectorT& getAdjointResidual() override
      {
        return fB_;
      }

      const VectorT& getAdjointResidual() const override
      {
        return fB_;
      }

      VectorT& getAdjointIntegrand() override
      {
        return gB_;
      }

      const VectorT& getAdjointIntegrand() const override
      {
        return gB_;
      }

      IdxT getIDcomponent()
      {
        return 0;
      }

    protected:
      void allocateVectors(IdxT n)
      {
        y_.resize(n);
        yp_.resize(n);
        f_.resize(n);
        abs_tol_.resize(n);
      }

      VectorT           y_;
      VectorT           yp_;
      std::vector<bool> tag_;
      VectorT           abs_tol_;
      VectorT           f_;
      VectorT           g_;

      VectorT yB_;
      VectorT ypB_;
      VectorT fB_;
      VectorT gB_;

      GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>* csr_jac_{nullptr};

      VectorT param_;
      VectorT param_up_;
      VectorT param_lo_;

      bool allocated_{false};
    };

    template <class ScalarT, typename IdxT>
    class AlgebraicErrorControlEvaluator : public NullEvaluator<ScalarT, IdxT>
    {
    protected:
      using NullEvaluator<ScalarT, IdxT>::allocated_;
      using NullEvaluator<ScalarT, IdxT>::y_;
      using NullEvaluator<ScalarT, IdxT>::yp_;
      using NullEvaluator<ScalarT, IdxT>::abs_tol_;
      using NullEvaluator<ScalarT, IdxT>::tag_;
      using NullEvaluator<ScalarT, IdxT>::f_;

    public:
      using RealT = typename NullEvaluator<ScalarT, IdxT>::RealT;

      int initialize() override
      {
        if (!allocated_)
        {
          this->allocate();
        }

        auto* y       = y_.getData();
        auto* yp      = yp_.getData();
        auto* abs_tol = abs_tol_.getData();
        auto* f       = f_.getData();

        y[0]       = 0.0;
        y[1]       = 0.0;
        yp[0]      = 0.0;
        yp[1]      = 0.0;
        tag_       = {true, false};
        abs_tol[0] = 0.0;
        abs_tol[1] = 0.0;
        f[0]       = 0.0;
        f[1]       = 0.0;
        t_         = 0.0;
        y_.setDataUpdated();
        yp_.setDataUpdated();
        abs_tol_.setDataUpdated();
        f_.setDataUpdated();
        return 0;
      }

      IdxT size() override
      {
        return 2;
      }

      int evaluateResidual() override
      {
        static constexpr RealT OMEGA = 100.0;
        auto*                  f     = f_.getData();
        const auto*            y     = y_.getData();
        const auto*            yp    = yp_.getData();

        f[0] = yp[0];
        f[1] = y[1] - std::sin(OMEGA * t_);
        f_.setDataUpdated();
        return 0;
      }

      void updateTime(RealT t, [[maybe_unused]] RealT a) override
      {
        t_ = t;
      }

    private:
      RealT t_{};
    };

    template <class ScalarT, typename IdxT>
    class ConsistentStateEvaluator : public NullEvaluator<ScalarT, IdxT>
    {
    protected:
      using NullEvaluator<ScalarT, IdxT>::allocated_;
      using NullEvaluator<ScalarT, IdxT>::csr_jac_;
      using NullEvaluator<ScalarT, IdxT>::f_;
      using NullEvaluator<ScalarT, IdxT>::tag_;
      using NullEvaluator<ScalarT, IdxT>::y_;
      using NullEvaluator<ScalarT, IdxT>::yp_;

    public:
      using RealT = typename NullEvaluator<ScalarT, IdxT>::RealT;

      ~ConsistentStateEvaluator() override
      {
        delete csr_jac_;
      }

      IdxT size() override
      {
        return 2;
      }

      int allocate() override
      {
        if (!allocated_)
        {
          NullEvaluator<ScalarT, IdxT>::allocate();
          auto* rows    = new IdxT[3]{0, 1, 3};
          auto* columns = new IdxT[3]{0, 0, 1};
          auto* values  = new RealT[3]{0.0, -2.0, 1.0};
          csr_jac_      = new GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>(
              2, 2, 3, &rows, &columns, &values);
        }
        return 0;
      }

      int initialize() override
      {
        ++initialize_calls_;
        return NullEvaluator<ScalarT, IdxT>::initialize();
      }

      int tagDifferentiable() override
      {
        tag_ = {true, false};
        return 0;
      }

      int evaluateResidual() override
      {
        auto*       f  = f_.getData();
        const auto* y  = y_.getData();
        const auto* yp = yp_.getData();

        f[0] = yp[0] - command_;
        f[1] = y[1] - RealT{2.0} * y[0] - command_;
        f_.setDataUpdated();
        return 0;
      }

      bool hasJacobian() override
      {
        return true;
      }

      int evaluateJacobian() override
      {
        auto* values = csr_jac_->getValues();
        values[0]    = alpha_;
        values[1]    = RealT{-2.0};
        values[2]    = RealT{1.0};
        return 0;
      }

      void updateTime([[maybe_unused]] RealT t, RealT alpha) override
      {
        alpha_ = alpha;
      }

      void seed(RealT differential_state, RealT algebraic_guess)
      {
        if (!allocated_)
        {
          this->allocate();
        }
        auto* y  = y_.getData();
        auto* yp = yp_.getData();
        y[0]     = differential_state;
        y[1]     = algebraic_guess;
        yp[0]    = RealT{0.0};
        yp[1]    = RealT{0.0};
        y_.setDataUpdated();
        yp_.setDataUpdated();
      }

      void setCommand(RealT command)
      {
        command_ = command;
      }

      int initializeCalls() const
      {
        return initialize_calls_;
      }

    private:
      RealT command_{0.0};
      RealT alpha_{0.0};
      int   initialize_calls_{0};
    };
  } // namespace Model

  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class IdaTests
    {
    public:
      TestOutcome callback()
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
        ida.runSimulation(1.0, 1.0 / n_steps, output_cb);

        success *= (observed_steps == n_steps);

        return success.report(__func__);
      }

      TestOutcome dtMonitorZero()
      {
        TestStatus success = true;

        Model::NullEvaluator<ScalarT, IdxT> model;

        Ida<double, size_t> ida(&model);
        ida.configureSimulation();

        unsigned observed_steps = 0;
        double   observed_t     = 0.0;
        auto     output_cb      = [&](double t)
        {
          observed_steps++;
          observed_t = t;
        };

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(1.0, 0.0, output_cb);

        success *= (observed_steps == 1);
        success *= (observed_t == 1.0);

        return success.report(__func__);
      }

      TestOutcome dtMonitorSuppressesEpsilonFinalStep()
      {
        TestStatus success = true;

        Model::NullEvaluator<ScalarT, IdxT> model;

        Ida<double, size_t> ida(&model);
        ida.configureSimulation();

        unsigned observed_steps = 0;
        double   observed_t     = 0.0;
        auto     output_cb      = [&](double t)
        {
          observed_steps++;
          observed_t = t;
        };

        const double tf = std::nextafter(1.0, 2.0);

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(tf, 0.25, output_cb);

        success *= (observed_steps == 4);
        success *= (observed_t == tf);

        return success.report(__func__);
      }

      TestOutcome fixedStep()
      {
        const unsigned n_steps = 32;
        TestStatus     success = true;

        Model::NullEvaluator<ScalarT, IdxT> model;

        Ida<double, size_t> ida(&model);
        ida.setFixedStep(1.0 / n_steps);
        ida.setTolerance(1.0e-6);
        ida.configureSimulation();

        ida.initializeSimulation(0.0, false);
        ida.runSimulation(1.0);
        auto stats = ida.getStats();

        success *= (stats.num_steps_ == n_steps);

        return success.report(__func__);
      }

      TestOutcome suppressAlgebraicErrors()
      {
        TestStatus success = true;

        const auto countSteps = [](bool suppress_alg)
        {
          Model::AlgebraicErrorControlEvaluator<ScalarT, IdxT> model;

          Ida<ScalarT, IdxT> ida(&model);
          ida.setSuppressAlgebraicErrors(suppress_alg);
          ida.setTolerance(1.0e-6);
          ida.setMaxSteps(10000);
          ida.configureSimulation();

          ida.initializeSimulation(0.0, false);
          ida.runSimulation(1.0);

          return ida.getStats().num_steps_;
        };

        const auto unsuppressed_steps = countSteps(false);
        const auto suppressed_steps   = countSteps(true);

        success *= (suppressed_steps < unsuppressed_steps);

        return success.report(__func__);
      }

      TestOutcome consistentStartAndRestart()
      {
        TestStatus success = true;

        Model::ConsistentStateEvaluator<ScalarT, IdxT> model;
        model.seed(2.0, -100.0);
        model.setCommand(1.0);

        Ida<ScalarT, IdxT> ida(&model);
        ida.setTolerance(1.0e-9, 1.0e-11);
        ida.configureSimulationFromCurrentState();

        bool rejected_nonforward_target{false};
        try
        {
          ida.startSimulation(0.0, 0.0);
        }
        catch (const std::invalid_argument&)
        {
          rejected_nonforward_target = true;
        }
        success *= rejected_nonforward_target;
        success *= (model.initializeCalls() == 0);

        ida.startSimulation(0.0, 0.1);
        success *= isEqual(model.y().getData()[0], 2.0, 1.0e-10);
        success *= isEqual(model.y().getData()[1], 5.0, 1.0e-10);
        success *= isEqual(model.yp().getData()[0], 1.0, 1.0e-10);

        ida.runSimulation(0.1);
        const auto state_at_event = model.y().getData()[0];

        model.setCommand(-2.0);
        bool rejected_mismatched_restart{false};
        try
        {
          ida.restartSimulation(0.11, 0.2);
        }
        catch (const std::invalid_argument&)
        {
          rejected_mismatched_restart = true;
        }
        success *= rejected_mismatched_restart;

        ida.restartSimulation(0.1, 0.2);
        success *= isEqual(model.y().getData()[0], state_at_event, 1.0e-9);
        success *= isEqual(model.y().getData()[1],
                           2.0 * state_at_event - 2.0,
                           1.0e-9);
        success *= isEqual(model.yp().getData()[0], -2.0, 1.0e-9);
        success *= (model.initializeCalls() == 0);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
