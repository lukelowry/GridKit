#pragma once

#include <algorithm>
#include <cmath>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::Bus(const ModelDataT& data)
      : bus_id_(data.bus_id),
        monitor_(std::make_unique<MonitorT>("Bus_" + data.name,
                                            data.monitored_variables))
    {
      size_ = 3;
      initializeMonitor();
    }

    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::~Bus() = default;

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::busID() const -> IdxT
    {
      return bus_id_;
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::setVoltageClass(BusVoltageClass voltage_class)
    {
      voltage_class_ = voltage_class;
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::voltageClass() const -> BusVoltageClass
    {
      return voltage_class_;
    }

    template <typename scalar_type, typename index_type>
    bool Bus<scalar_type, index_type>::differentiatedKCL() const
    {
      return kcl_differentiation_required_ && !original_kcl_validation_;
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::setKCLDifferentiationRequired(
        bool required)
    {
      kcl_differentiation_required_ = required;
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::setOriginalKCLValidation(bool active)
    {
      original_kcl_validation_ = active;
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Va() -> ScalarT&
    {
      return y_.getData()[0];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Vb() -> ScalarT&
    {
      return y_.getData()[1];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Vc() -> ScalarT&
    {
      return y_.getData()[2];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Vap() -> ScalarT&
    {
      return yp_.getData()[0];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Vbp() -> ScalarT&
    {
      return yp_.getData()[1];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Vcp() -> ScalarT&
    {
      return yp_.getData()[2];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Ia() -> ScalarT&
    {
      return f_.getData()[0];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Ib() -> ScalarT&
    {
      return f_.getData()[1];
    }

    template <typename scalar_type, typename index_type>
    auto Bus<scalar_type, index_type>::Ic() -> ScalarT&
    {
      return f_.getData()[2];
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::accumulateCurrent(const ScalarT& ia,
                                                         const ScalarT& ib,
                                                         const ScalarT& ic)
    {
      const ABCVector<ScalarT> current{ia, ib, ic};
      auto*                    residual = f_.getData();
      for (std::size_t phase = 0; phase < current.size(); ++phase)
      {
        residual[phase] += current[phase];
        current_scale_[phase] +=
            std::abs(Detail::scalarValue<RealT>(current[phase]));
      }
      f_.setDataUpdated();
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::allocate()
    {
      if (!allocated_)
      {
        this->allocateVectors(size_);
      }

      tag_.resize(3);
      variable_indices_.resize(3);
      residual_indices_.resize(3);
      this->setVariableIndex(0, 0);
      this->setVariableIndex(1, 1);
      this->setVariableIndex(2, 2);
      this->setResidualIndex(0, 0);
      this->setResidualIndex(1, 1);
      this->setResidualIndex(2, 2);
      allocated_ = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::verify() const
    {
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::initialize()
    {
      auto* y  = y_.getData();
      auto* yp = yp_.getData();
      y[0]     = ScalarT{0.0};
      y[1]     = ScalarT{0.0};
      y[2]     = ScalarT{0.0};
      yp[0]    = ScalarT{0.0};
      yp[1]    = ScalarT{0.0};
      yp[2]    = ScalarT{0.0};
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const
    {
      variables.push_back({"v", std::nullopt, 0});
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::tagDifferentiable()
    {
      const bool differential = voltage_class_ == BusVoltageClass::differential;
      tag_[0]                 = differential;
      tag_[1]                 = differential;
      tag_[2]                 = differential;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::setAbsoluteTolerance(RealT absolute_tolerance)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(absolute_tolerance));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::evaluateResidual()
    {
      Ia() = ScalarT{0.0};
      Ib() = ScalarT{0.0};
      Ic() = ScalarT{0.0};
      current_scale_.fill(RealT{0.0});
      f_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    const Model::VariableMonitorBase* Bus<scalar_type, index_type>::getMonitor() const
    {
      return monitor_.get();
    }

    template <typename scalar_type, typename index_type>
    void Bus<scalar_type, index_type>::initializeMonitor()
    {
      using Variable = typename ModelDataT::MonitorableVariables;
      monitor_->set(Variable::va, [this]
                    { return Va(); });
      monitor_->set(Variable::vb, [this]
                    { return Vb(); });
      monitor_->set(Variable::vc, [this]
                    { return Vc(); });
    }
  } // namespace EMT
} // namespace GridKit
