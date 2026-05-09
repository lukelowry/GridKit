/**
 * @file SystemModel.hpp
 * @brief Minimal EMT system model with owned buses and stamping components.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Line/Line.hpp>
#include <GridKit/Model/EMT/Load/ShuntLoad.hpp>
#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/Evaluator.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class SystemModel : public Model::Evaluator<ScalarT, IdxT>
    {
    public:
      using RealT              = typename Model::Evaluator<ScalarT, IdxT>::RealT;
      using MatrixT            = typename Model::Evaluator<ScalarT, IdxT>::MatrixT;
      using CsrMatrixT         = typename Model::Evaluator<ScalarT, IdxT>::CsrMatrixT;
      using BusDataT           = BusData<RealT, IdxT>;
      using LineDataT          = LineData<RealT, IdxT>;
      using ShuntLoadDataT     = ShuntLoadData<RealT, IdxT>;
      using VoltageSourceDataT = VoltageSourceData<RealT, IdxT>;
      using SystemModelDataT   = SystemModelData<RealT, IdxT>;
      using BusT               = Bus<ScalarT, IdxT>;
      using LineT              = Line<ScalarT, IdxT>;
      using ShuntLoadT         = ShuntLoad<ScalarT, IdxT>;
      using VoltageSourceT     = VoltageSource<ScalarT, IdxT>;
      using ComponentT         = Component<ScalarT, IdxT>;
      using TerminalRef        = typename ComponentT::TerminalRef;

      SystemModel() = default;
      explicit SystemModel(const SystemModelDataT& data);
      ~SystemModel() override = default;

      BusT&           addBus(const BusDataT& data);
      LineT&          addLine(const LineDataT& data);
      ShuntLoadT&     addShuntLoad(const ShuntLoadDataT& data);
      VoltageSourceT& addVoltageSource(const VoltageSourceDataT& data);
      int             connect(TerminalRef terminal, BusT& bus);

      LineT*                getLine(const std::string& id);
      const LineT*          getLine(const std::string& id) const;
      ShuntLoadT*           getShuntLoad(const std::string& id);
      const ShuntLoadT*     getShuntLoad(const std::string& id) const;
      VoltageSourceT*       getVoltageSource(const std::string& id);
      const VoltageSourceT* getVoltageSource(const std::string& id) const;
      void                  cue(const std::string& target, Action action);

      int allocate() override final;
      int verify() const;
      int initialize() override final;
      int tagDifferentiable() override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      bool hasJacobian() override final;
      void updateTime(RealT t, RealT a) override final;

      IdxT size() override final;
      IdxT nnz() override final;
      IdxT sizeQuadrature() override final;
      IdxT sizeParams() override final;

      void setTolerances(RealT& rtol, RealT& atol) const override final;
      void setMaxSteps(IdxT& msa) const override final;

      std::vector<ScalarT>&       y() override final;
      const std::vector<ScalarT>& y() const override final;
      std::vector<ScalarT>&       yp() override final;
      const std::vector<ScalarT>& yp() const override final;
      std::vector<bool>&          tag() override final;
      const std::vector<bool>&    tag() const override final;

      std::vector<ScalarT>&       yB() override final;
      const std::vector<ScalarT>& yB() const override final;
      std::vector<ScalarT>&       ypB() override final;
      const std::vector<ScalarT>& ypB() const override final;

      std::vector<ScalarT>&       param() override final;
      const std::vector<ScalarT>& param() const override final;
      std::vector<ScalarT>&       param_up() override final;
      const std::vector<ScalarT>& param_up() const override final;
      std::vector<ScalarT>&       param_lo() override final;
      const std::vector<ScalarT>& param_lo() const override final;

      std::vector<ScalarT>&       getResidual() override final;
      const std::vector<ScalarT>& getResidual() const override final;
      MatrixT&                    getJacobian() override final;
      const MatrixT&              getJacobian() const override final;
      CsrMatrixT*                 getCsrJacobian() const override final;

      int                         evaluateIntegrand() override final;
      std::vector<ScalarT>&       getIntegrand() override final;
      const std::vector<ScalarT>& getIntegrand() const override final;

      int                         initializeAdjoint() override final;
      int                         evaluateAdjointResidual() override final;
      int                         evaluateAdjointIntegrand() override final;
      std::vector<ScalarT>&       getAdjointResidual() override final;
      const std::vector<ScalarT>& getAdjointResidual() const override final;
      std::vector<ScalarT>&       getAdjointIntegrand() override final;
      const std::vector<ScalarT>& getAdjointIntegrand() const override final;

    private:
      void ensureMutableTopology() const;
      void allocateVectors();
      int  verifyConnections() const;
      int  bindConnections();
      void distributeVectors();
      void assembleTerminalCurrents(const ComponentT& component);
      void stampTerminalDifferentiability(const ComponentT& component);
      void appendJacobianEntries(ComponentT& component);
      void assembleCsrFromCurrentJacobians();
      void updateCsrValuesFromCurrentJacobians();

    private:
      struct Connection
      {
        TerminalRef terminal;
        BusT*       bus{nullptr};
      };

      std::vector<std::unique_ptr<BusT>>       buses_;
      std::vector<std::unique_ptr<ComponentT>> components_;
      std::vector<Connection>                  connections_;
      std::vector<LineT*>                      lines_;
      std::vector<std::string>                 line_ids_;
      std::vector<ShuntLoadT*>                 shunt_loads_;
      std::vector<std::string>                 shunt_load_ids_;
      std::vector<VoltageSourceT*>             voltage_sources_;
      std::vector<std::string>                 voltage_source_ids_;

      bool allocated_{false};

      IdxT size_{0};
      IdxT nnz_{0};

      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<bool>    tag_;
      std::vector<ScalarT> f_;
      std::vector<ScalarT> g_;

      MatrixT                     J_;
      std::unique_ptr<CsrMatrixT> csr_jac_;
      std::vector<IdxT>           map_to_csr_;

      RealT time_{0.0};
      RealT alpha_{0.0};
      RealT rel_tol_{1.0e-6};
      RealT abs_tol_{1.0e-8};
      IdxT  max_steps_{2000};

      std::vector<ScalarT> yB_;
      std::vector<ScalarT> ypB_;
      std::vector<ScalarT> fB_;
      std::vector<ScalarT> gB_;
      std::vector<ScalarT> param_;
      std::vector<ScalarT> param_up_;
      std::vector<ScalarT> param_lo_;
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/SystemModelImpl.hpp>
