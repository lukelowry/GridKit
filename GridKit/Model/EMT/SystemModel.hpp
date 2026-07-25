#pragma once

#include <map>
#include <memory>
#include <numbers>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>

namespace GridKit
{
  namespace Model
  {
    template <typename scalar_type>
    class VariableMonitorController;
  }

  namespace PhasorDynamics
  {
    template <typename scalar_type, typename index_type>
    class SignalNode;
  }

  namespace EMT
  {
    template <typename real_type, typename index_type>
    struct SystemModelData;

    template <typename scalar_type, typename index_type>
    class Bus;

    template <typename scalar_type, typename index_type>
    class SystemModel
      : public PhasorDynamics::Component<scalar_type, index_type>
    {
      using PhasorDynamics::Component<scalar_type, index_type>::gridkit_component_id_;
      using PhasorDynamics::Component<scalar_type, index_type>::size_;
      using PhasorDynamics::Component<scalar_type, index_type>::nnz_;
      using PhasorDynamics::Component<scalar_type, index_type>::time_;
      using PhasorDynamics::Component<scalar_type, index_type>::alpha_;
      using PhasorDynamics::Component<scalar_type, index_type>::y_;
      using PhasorDynamics::Component<scalar_type, index_type>::yp_;
      using PhasorDynamics::Component<scalar_type, index_type>::tag_;
      using PhasorDynamics::Component<scalar_type, index_type>::abs_tol_;
      using PhasorDynamics::Component<scalar_type, index_type>::f_;
      using PhasorDynamics::Component<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::residual_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::csr_jac_;
      using PhasorDynamics::Component<scalar_type, index_type>::map_to_csr_;
      using PhasorDynamics::Component<scalar_type, index_type>::allocated_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using RealT      = typename Model::Evaluator<ScalarT, IdxT>::RealT;
      using CsrMatrixT = typename Model::Evaluator<ScalarT, IdxT>::CsrMatrixT;
      using CooMatrixT = typename Model::Evaluator<ScalarT, IdxT>::CooMatrixT;
      using BusT       = Bus<ScalarT, IdxT>;
      using SignalT    = PhasorDynamics::SignalNode<ScalarT, IdxT>;
      using ComponentT = PhasorDynamics::Component<ScalarT, IdxT>;
      using MonitorT   = Model::VariableMonitorController<ScalarT>;

      SystemModel();
      explicit SystemModel(const SystemModelData<RealT, IdxT>& data);
      virtual ~SystemModel();

      int  setGridKitComponentID(IdxT component_id) override;
      int  allocate() override;
      int  verify() const override;
      int  initialize() override;
      bool hasJacobian() override;

      void initializeMonitor();
      void startMonitor() override;
      void stopMonitor() override;
      bool monitoring() const override;
      void printMonitoredVariables() const override;

      int  tagDifferentiable() override;
      int  setAbsoluteTolerance(RealT rel_tol) override;
      int  evaluateResidual() override;
      int  evaluateJacobian() override;
      void updateTime(RealT time, RealT alpha) override;

      void addBus(BusT* bus);
      void addSignal(SignalT* signal);
      void addComponent(ComponentT* component);

      BusT*       getBus(IdxT bus_id);
      SignalT*    getSignal(IdxT signal_id);
      ComponentT* getComponent(IdxT gridkit_component_id);

    private:
      using ComponentT::setSystemBase;

      std::vector<BusT*>       buses_;
      std::vector<SignalT*>    signals_;
      std::vector<ComponentT*> components_;

      std::map<IdxT, IdxT> gridkit_bus_indices_;
      std::map<IdxT, IdxT> gridkit_signal_indices_;

      bool  owns_components_{false};
      RealT omega0_{2.0 * std::numbers::pi_v<RealT> * 60.0};

      std::unique_ptr<MonitorT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
