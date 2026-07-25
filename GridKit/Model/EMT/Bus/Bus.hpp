#pragma once

#include <memory>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    class Bus final : public PhasorDynamics::Component<scalar_type, index_type>
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
      using ModelDataT = BusData<RealT, IdxT>;
      using MonitorT   = Model::VariableMonitor<Bus, BusData>;

      Bus(const ModelDataT&, RealT omega0);
      ~Bus() override;

      IdxT busID() const;

      ScalarT& Va();
      ScalarT& Vb();
      ScalarT& Vc();
      ScalarT& Vap();
      ScalarT& Vbp();
      ScalarT& Vcp();
      ScalarT& Ia();
      ScalarT& Ib();
      ScalarT& Ic();

      int setGridKitComponentID(IdxT) override final;
      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      const Model::VariableMonitorBase* getMonitor() const override;

    private:
      void initializeMonitor();

      IdxT                      bus_id_{0};
      RealT                     omega0_{0.0};
      std::complex<RealT>       Va0_{0.0, 0.0};
      std::complex<RealT>       Vb0_{0.0, 0.0};
      std::complex<RealT>       Vc0_{0.0, 0.0};
      std::unique_ptr<MonitorT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
