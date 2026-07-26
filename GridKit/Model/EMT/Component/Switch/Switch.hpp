#pragma once

#include <memory>
#include <vector>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/BusVoltageContribution.hpp>
#include <GridKit/Model/EMT/Component/Switch/SwitchData.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class SwitchInternalVariables : size_t
    {
      MAXIMUM
    };

    enum class SwitchExternalVariables : size_t
    {
      open,
      MAXIMUM
    };

    template <typename scalar_type, typename index_type>
    class Switch final : public PhasorDynamics::Component<scalar_type, index_type>,
                         public BusVoltageContributor<scalar_type, index_type>
    {
      using PhasorDynamics::Component<scalar_type, index_type>::abs_tol_;
      using PhasorDynamics::Component<scalar_type, index_type>::allocated_;
      using PhasorDynamics::Component<scalar_type, index_type>::f_;
      using PhasorDynamics::Component<scalar_type, index_type>::gridkit_component_id_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_cols_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_rows_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::J_vals_buffer_;
      using PhasorDynamics::Component<scalar_type, index_type>::nnz_;
      using PhasorDynamics::Component<scalar_type, index_type>::residual_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::size_;
      using PhasorDynamics::Component<scalar_type, index_type>::tag_;
      using PhasorDynamics::Component<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::y_;
      using PhasorDynamics::Component<scalar_type, index_type>::yp_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using RealT      = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ModelDataT = SwitchData<RealT, IdxT>;
      using MonitorT   = Model::VariableMonitor<Switch, SwitchData>;
      using SignalsT   = PhasorDynamics::ComponentSignals<ScalarT,
                                                          IdxT,
                                                          SwitchInternalVariables,
                                                          SwitchExternalVariables>;

      Switch(BusT* bus1, BusT* bus2, const ModelDataT&);
      ~Switch() override;

      int setGridKitComponentID(IdxT) override final;
      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      void appendBusVoltageContributions(
          std::vector<BusVoltageContribution<ScalarT, IdxT>>&) const override;

      SignalsT&                         getSignals();
      const Model::VariableMonitorBase* getMonitor() const override;

    private:
      void  initializeParameters(const ModelDataT&);
      void  initializeMonitor();
      RealT openCommand() const;

      BusT* bus1_{nullptr};
      BusT* bus2_{nullptr};
      IdxT  N_{3};

      SignalsT                  signals_;
      std::unique_ptr<MonitorT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
