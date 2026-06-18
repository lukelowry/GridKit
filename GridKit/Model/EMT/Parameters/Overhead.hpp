/**
 * @file Overhead.hpp
 *
 * @brief Aggregate frequency-domain model for an overhead transmission line. It
 * implements Model::Evaluator so IDA can evaluate it over angular frequency,
 * owns the global state, and wires its parameter elements per the Parameters
 * dependency diagram.
 *
 */

#pragma once

#include <memory>
#include <vector>

#include <GridKit/Model/EMT/Element.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/Carson/Carson.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/GeometricInductance/GeometricInductance.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/SeriesImpedance/SeriesImpedance.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/ShuntAdmittance/ShuntAdmittance.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/ShuntPotential/ShuntPotential.hpp>
#include <GridKit/Model/EMT/Parameters/Effects/SkinEffect/SkinEffect.hpp>
#include <GridKit/Model/EMT/Parameters/Geometry/Conductor/Conductor.hpp>
#include <GridKit/Model/EMT/Parameters/Geometry/Path/Path.hpp>
#include <GridKit/Model/EMT/Parameters/Geometry/Tower/Tower.hpp>
#include <GridKit/Model/EMT/Parameters/OverheadData.hpp>
#include <GridKit/Model/EMT/Parameters/Response/Gamma/Gamma.hpp>
#include <GridKit/Model/EMT/Parameters/Response/H/H.hpp>
#include <GridKit/Model/EMT/Parameters/Response/Tau/Tau.hpp>
#include <GridKit/Model/EMT/Parameters/Response/Yc/Yc.hpp>
#include <GridKit/Model/EMT/Parameters/Response/Zc/Zc.hpp>
#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Model/VariableMonitor.hpp>
#include <GridKit/Utilities/Errors.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Parameters
    {
      template <typename scalar_type, typename index_type>
      class Overhead : public Model::Evaluator<scalar_type, index_type>
      {
      public:
        using ScalarT            = scalar_type;
        using IdxT               = index_type;
        using RealT              = typename Model::Evaluator<ScalarT, IdxT>::RealT;
        using MatrixT            = typename Model::Evaluator<ScalarT, IdxT>::MatrixT;
        using CsrMatrixT         = typename Model::Evaluator<ScalarT, IdxT>::CsrMatrixT;
        using ElementT           = Element<ScalarT, IdxT>;
        using MonitorControllerT = Model::VariableMonitorController<ScalarT>;

        explicit Overhead(const OverheadData<ScalarT, IdxT>& data);
        ~Overhead() override;

        Overhead(const Overhead&)            = delete;
        Overhead& operator=(const Overhead&) = delete;

        int allocate() override;
        int initialize() override;
        int tagDifferentiable() override;
        int evaluateResidual() override;
        int evaluateJacobian() override;

        void                              updateTime(RealT t, RealT a) override;
        bool                              monitoring() const override;
        void                              startMonitor() override;
        void                              stopMonitor() override;
        void                              printMonitoredVariables() const override;
        const Model::VariableMonitorBase* getMonitor() const override;

        const Tower<ScalarT, IdxT>& tower() const
        {
          return tower_;
        }

        const Conductor<ScalarT, IdxT>& conductor() const
        {
          return conductor_;
        }

        const Path<ScalarT, IdxT>& path() const
        {
          return path_;
        }

        const GeometricInductance<ScalarT, IdxT>& geometricInductance() const
        {
          return geometric_inductance_;
        }

        const SkinEffect<ScalarT, IdxT>& skinEffect() const
        {
          return skin_effect_;
        }

        const Carson<ScalarT, IdxT>& carson() const
        {
          return carson_;
        }

        const SeriesImpedance<ScalarT, IdxT>& seriesImpedance() const
        {
          return series_impedance_;
        }

        const ShuntPotential<ScalarT, IdxT>& shuntPotential() const
        {
          return shunt_potential_;
        }

        const ShuntAdmittance<ScalarT, IdxT>& shuntAdmittance() const
        {
          return shunt_admittance_;
        }

        const Gamma<ScalarT, IdxT>& gamma() const
        {
          return gamma_;
        }

        const Tau<ScalarT, IdxT>& tau() const
        {
          return tau_;
        }

        const H<ScalarT, IdxT>& h() const
        {
          return h_;
        }

        const Yc<ScalarT, IdxT>& yc() const
        {
          return yc_;
        }

        const Zc<ScalarT, IdxT>& zc() const
        {
          return zc_;
        }

        RealT omega() const
        {
          return omega_;
        }

        IdxT size() override
        {
          return size_;
        }

        IdxT nnz() override
        {
          return nnz_;
        }

        bool hasJacobian() override
        {
          return true;
        }

        CsrMatrixT* getCsrJacobian() const override
        {
          return csr_jac_;
        }

        void setTolerances(RealT& rtol, RealT& atol) const override
        {
          rtol = rel_tol_;
          atol = abs_tol_;
        }

        void setMaxSteps(IdxT& msa) const override
        {
          msa = max_steps_;
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

        std::vector<ScalarT>& getResidual() override
        {
          return f_;
        }

        const std::vector<ScalarT>& getResidual() const override
        {
          return f_;
        }

        MatrixT& getJacobian() override
        {
          return jac_;
        }

        const MatrixT& getJacobian() const override
        {
          return jac_;
        }

        // The frequency-sweep aggregate does not provide quadrature, adjoint, or
        // parameter sensitivity interfaces.
        [[noreturn]] int evaluateIntegrand() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] int initializeAdjoint() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] int evaluateAdjointResidual() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] int evaluateAdjointIntegrand() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] IdxT sizeQuadrature() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] IdxT sizeParams() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& yB() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& yB() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& ypB() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& ypB() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& param() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& param() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& param_up() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& param_up() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& param_lo() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& param_lo() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& getIntegrand() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& getIntegrand() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& getAdjointResidual() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& getAdjointResidual() const override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] std::vector<ScalarT>& getAdjointIntegrand() override
        {
          throw NotImplementedError(__func__);
        }

        [[noreturn]] const std::vector<ScalarT>& getAdjointIntegrand() const override
        {
          throw NotImplementedError(__func__);
        }

      private:
        using NotImplementedError = GridKit::Utilities::NotImplementedError;

        void initializeMonitor();

        Conductor<ScalarT, IdxT>           conductor_;
        Tower<ScalarT, IdxT>               tower_;
        Path<ScalarT, IdxT>                path_;
        GeometricInductance<ScalarT, IdxT> geometric_inductance_;
        SkinEffect<ScalarT, IdxT>          skin_effect_;
        Carson<ScalarT, IdxT>              carson_;
        SeriesImpedance<ScalarT, IdxT>     series_impedance_;
        ShuntPotential<ScalarT, IdxT>      shunt_potential_;
        ShuntAdmittance<ScalarT, IdxT>     shunt_admittance_;
        Gamma<ScalarT, IdxT>               gamma_;
        Tau<ScalarT, IdxT>                 tau_;
        H<ScalarT, IdxT>                   h_;
        Yc<ScalarT, IdxT>                  yc_;
        Zc<ScalarT, IdxT>                  zc_;
        std::vector<ElementT*>             elements_;

        std::vector<ScalarT> y_;
        std::vector<ScalarT> yp_;
        std::vector<ScalarT> f_;
        std::vector<bool>    tag_;
        std::vector<IdxT>    gidx_;

        IdxT size_{0};
        IdxT nnz_{0};

        RealT omega_{0.0};
        RealT alpha_{0.0};
        RealT rel_tol_{1.0e-7};
        RealT abs_tol_{1.0e-9};
        IdxT  max_steps_{200000};

        MatrixT           jac_;
        CsrMatrixT*       csr_jac_{nullptr};
        std::vector<IdxT> csr_map_;

        std::unique_ptr<Model::VariableMonitorBase> monitor_;
        std::unique_ptr<MonitorControllerT>         monitor_controller_;
        bool                                        monitor_initialized_{false};
      };
    } // namespace Parameters
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Parameters/OverheadImpl.hpp>
