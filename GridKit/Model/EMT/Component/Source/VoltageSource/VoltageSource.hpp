#pragma once

#include <memory>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/BusVoltageContribution.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceData.hpp>
#include <GridKit/Model/EMT/InitialStateLayout.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    class VoltageSource final : public PhasorDynamics::Component<scalar_type, index_type>,
                                public BusVoltageContributor<scalar_type, index_type>,
                                public InitialStateLayout
    {
      using PhasorDynamics::Component<scalar_type, index_type>::abs_tol_;
      using PhasorDynamics::Component<scalar_type, index_type>::allocated_;
      using PhasorDynamics::Component<scalar_type, index_type>::alpha_;
      using PhasorDynamics::Component<scalar_type, index_type>::f_;
      using PhasorDynamics::Component<scalar_type, index_type>::gridkit_component_id_;
      using PhasorDynamics::Component<scalar_type, index_type>::h_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_cols_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_rows_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_vals_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::nnz_;
      using PhasorDynamics::Component<scalar_type, index_type>::residual_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::size_;
      using PhasorDynamics::Component<scalar_type, index_type>::tag_;
      using PhasorDynamics::Component<scalar_type, index_type>::time_;
      using PhasorDynamics::Component<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::wb_;
      using PhasorDynamics::Component<scalar_type, index_type>::y_;
      using PhasorDynamics::Component<scalar_type, index_type>::yp_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using RealT      = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ModelDataT = VoltageSourceData<RealT, IdxT>;
      using MonitorT   = Model::VariableMonitor<VoltageSource, VoltageSourceData>;

      VoltageSource(BusT*, const ModelDataT&);
      ~VoltageSource() override;

      int setGridKitComponentID(IdxT) override final;
      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      void appendInitialStateVariables(
          std::vector<InitialStateVariable>&) const override;
      void appendBusVoltageContributions(
          std::vector<BusVoltageContribution<ScalarT, IdxT>>&) const override;

      const Model::VariableMonitorBase* getMonitor() const override;

      __attribute__((always_inline)) inline int evaluateInternalResidual(
          const ScalarT* y,
          const ScalarT* yp,
          const ScalarT* wb,
          ScalarT*       f);

      __attribute__((always_inline)) inline int evaluateBusResidual(
          const ScalarT* y,
          ScalarT*       h);

    private:
      void initializeParameters(const ModelDataT&);
      void initializeMonitor();

      BusT* bus_{nullptr};

      ABCVector<RealT> E_{};
      ABCVector<RealT> phi_{};
      RealT            omega_{0.0};
      ABCMatrix<RealT> Rs_{};
      ABCMatrix<RealT> Ls_{};

      std::unique_ptr<MonitorT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
