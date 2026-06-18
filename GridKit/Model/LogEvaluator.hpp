#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Utilities/Errors.hpp>

namespace GridKit
{
  namespace Model
  {
    template <class ScalarT, typename IdxT>
    class LogEvaluator final : public Evaluator<ScalarT, IdxT>
    {
    public:
      using BaseT      = Evaluator<ScalarT, IdxT>;
      using RealT      = typename BaseT::RealT;
      using MatrixT    = typename BaseT::MatrixT;
      using CsrMatrixT = typename BaseT::CsrMatrixT;

      LogEvaluator(BaseT& model, RealT omega)
        : model_(model)
      {
        if (omega <= 0.0)
        {
          throw std::invalid_argument("LogEvaluator requires a positive coordinate");
        }

        omega_ = omega;
        s_     = std::log(omega_);
      }

      int allocate() override
      {
        model_.updateTime(omega_, 1.0);
        const int retval = model_.allocate();
        syncSolverDerivativeFromPhysical();
        return retval;
      }

      int initialize() override
      {
        model_.updateTime(omega_, 0.0);
        const int retval = model_.initialize();
        syncSolverDerivativeFromPhysical();
        return retval;
      }

      int tagDifferentiable() override
      {
        return model_.tagDifferentiable();
      }

      int evaluateResidual() override
      {
        return model_.evaluateResidual();
      }

      int evaluateJacobian() override
      {
        return model_.evaluateJacobian();
      }

      int evaluateIntegrand() override
      {
        const int retval = model_.evaluateIntegrand();
        integrand_log_   = model_.getIntegrand();
        for (auto& value : integrand_log_)
        {
          value *= omega_;
        }
        return retval;
      }

      int initializeAdjoint() override
      {
        throw Utilities::NotImplementedError(__func__);
      }

      int evaluateAdjointResidual() override
      {
        throw Utilities::NotImplementedError(__func__);
      }

      int evaluateAdjointIntegrand() override
      {
        throw Utilities::NotImplementedError(__func__);
      }

      IdxT size() override
      {
        return model_.size();
      }

      IdxT nnz() override
      {
        return model_.nnz();
      }

      bool monitoring() const override
      {
        return model_.monitoring();
      }

      void printMonitoredVariables() const override
      {
        model_.printMonitoredVariables();
      }

      const VariableMonitorBase* getMonitor() const override
      {
        return model_.getMonitor();
      }

      void startMonitor() override
      {
        model_.startMonitor();
      }

      void stopMonitor() override
      {
        model_.stopMonitor();
      }

      CsrMatrixT* getCsrJacobian() const override
      {
        return model_.getCsrJacobian();
      }

      bool hasJacobian() override
      {
        return model_.hasJacobian();
      }

      IdxT sizeQuadrature() override
      {
        return model_.sizeQuadrature();
      }

      IdxT sizeParams() override
      {
        return model_.sizeParams();
      }

      void updateTime(RealT s, RealT alpha) override
      {
        s_     = s;
        omega_ = std::exp(s_);

        syncPhysicalDerivativeFromSolver();
        model_.updateTime(omega_, alpha / omega_);
      }

      void setTolerances(RealT& rtol, RealT& atol) const override
      {
        model_.setTolerances(rtol, atol);
      }

      void setMaxSteps(IdxT& msa) const override
      {
        model_.setMaxSteps(msa);
      }

      std::vector<ScalarT>& y() override
      {
        return model_.y();
      }

      const std::vector<ScalarT>& y() const override
      {
        return model_.y();
      }

      std::vector<ScalarT>& yp() override
      {
        return yp_log_;
      }

      const std::vector<ScalarT>& yp() const override
      {
        return yp_log_;
      }

      std::vector<bool>& tag() override
      {
        return model_.tag();
      }

      const std::vector<bool>& tag() const override
      {
        return model_.tag();
      }

      std::vector<ScalarT>& yB() override
      {
        return model_.yB();
      }

      const std::vector<ScalarT>& yB() const override
      {
        return model_.yB();
      }

      std::vector<ScalarT>& ypB() override
      {
        return model_.ypB();
      }

      const std::vector<ScalarT>& ypB() const override
      {
        return model_.ypB();
      }

      std::vector<ScalarT>& param() override
      {
        return model_.param();
      }

      const std::vector<ScalarT>& param() const override
      {
        return model_.param();
      }

      std::vector<ScalarT>& param_up() override
      {
        return model_.param_up();
      }

      const std::vector<ScalarT>& param_up() const override
      {
        return model_.param_up();
      }

      std::vector<ScalarT>& param_lo() override
      {
        return model_.param_lo();
      }

      const std::vector<ScalarT>& param_lo() const override
      {
        return model_.param_lo();
      }

      std::vector<ScalarT>& getResidual() override
      {
        return model_.getResidual();
      }

      const std::vector<ScalarT>& getResidual() const override
      {
        return model_.getResidual();
      }

      MatrixT& getJacobian() override
      {
        return model_.getJacobian();
      }

      const MatrixT& getJacobian() const override
      {
        return model_.getJacobian();
      }

      std::vector<ScalarT>& getIntegrand() override
      {
        return integrand_log_;
      }

      const std::vector<ScalarT>& getIntegrand() const override
      {
        return integrand_log_;
      }

      std::vector<ScalarT>& getAdjointResidual() override
      {
        return model_.getAdjointResidual();
      }

      const std::vector<ScalarT>& getAdjointResidual() const override
      {
        return model_.getAdjointResidual();
      }

      std::vector<ScalarT>& getAdjointIntegrand() override
      {
        return model_.getAdjointIntegrand();
      }

      const std::vector<ScalarT>& getAdjointIntegrand() const override
      {
        return model_.getAdjointIntegrand();
      }

      RealT omega() const
      {
        return omega_;
      }

      RealT coordinate() const
      {
        return s_;
      }

    private:
      void syncPhysicalDerivativeFromSolver()
      {
        auto& yp_coordinate = model_.yp();
        yp_coordinate.resize(yp_log_.size());

        for (size_t i = 0; i < yp_log_.size(); ++i)
        {
          yp_coordinate[i] = yp_log_[i] / omega_;
        }
      }

      void syncSolverDerivativeFromPhysical()
      {
        const auto& yp_coordinate = model_.yp();
        yp_log_.resize(yp_coordinate.size());

        for (size_t i = 0; i < yp_log_.size(); ++i)
        {
          yp_log_[i] = omega_ * yp_coordinate[i];
        }
      }

      BaseT&               model_;
      RealT                omega_{1.0};
      RealT                s_{0.0};
      std::vector<ScalarT> yp_log_;
      std::vector<ScalarT> integrand_log_;
    };
  } // namespace Model
} // namespace GridKit
