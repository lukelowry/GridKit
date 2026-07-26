#pragma once

#include <memory>
#include <vector>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/BusVoltageContribution.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedData.hpp>
#include <GridKit/Model/EMT/InitialStateLayout.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    class SystemModel;

    template <typename scalar_type, typename index_type>
    class LineLumped final
      : public PhasorDynamics::Component<scalar_type, index_type>,
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
      using PhasorDynamics::Component<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::wb_;
      using PhasorDynamics::Component<scalar_type, index_type>::y_;
      using PhasorDynamics::Component<scalar_type, index_type>::yp_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using RealT      = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ModelDataT = LineLumpedData<RealT, IdxT>;
      using MonitorT   = Model::VariableMonitor<LineLumped, LineLumpedData>;

      LineLumped(BusT* bus1, BusT* bus2, const ModelDataT&);
      ~LineLumped() override;

      int setGridKitComponentID(IdxT) override final;
      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      void appendBusVoltageContributions(
          std::vector<BusVoltageContribution<ScalarT, IdxT>>& contributions) const override;
      void appendInitialStateVariables(
          std::vector<InitialStateVariable>&) const override;

      const Model::VariableMonitorBase* getMonitor() const override;

      __attribute__((always_inline)) inline int evaluateInternalResidual(
          const ScalarT* y,
          const ScalarT* yp,
          const ScalarT* wb,
          const ScalarT* wbp,
          ScalarT*       f);

      __attribute__((always_inline)) inline int evaluateBus1Residual(
          const ScalarT* y,
          ScalarT*       h);

      __attribute__((always_inline)) inline int evaluateBus2Residual(
          const ScalarT* y,
          ScalarT*       h);

    private:
      friend class SystemModel<ScalarT, IdxT>;

      void initializeParameters(const ModelDataT&);
      void initializeMonitor();

      BusT* bus1_{nullptr};
      BusT* bus2_{nullptr};

      IdxT            N_{3};
      IdxT            K_{3};
      ABCVector<IdxT> conductors_{1, 2, 3};
      RealT           dx_{0.0};
      bool            zp_dynamic_{false};
      bool            yp_dynamic_{false};

      ABCMatrix<RealT> Rp_{};
      ABCMatrix<RealT> Lp_{};
      ABCMatrix<RealT> Gp_{};
      ABCMatrix<RealT> Cp_{};
      ABCMatrix<RealT> R_{};
      ABCMatrix<RealT> L_{};
      ABCMatrix<RealT> G_{};
      ABCMatrix<RealT> C_{};

      std::vector<ScalarT>      wbp_;
      std::vector<IdxT>         bus_variable_indices_;
      std::unique_ptr<MonitorT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
