#pragma once

#include <cmath>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    Bus<scalar_type, index_type>::Bus(const ModelDataT& data, RealT omega0)
      : bus_id_(data.bus_id),
        omega0_(omega0),
        Va0_(data.Va0),
        Vb0_(data.Vb0),
        Vc0_(data.Vc0),
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
      const bool finite_phasors = std::isfinite(Va0_.real()) && std::isfinite(Va0_.imag())
                                  && std::isfinite(Vb0_.real()) && std::isfinite(Vb0_.imag())
                                  && std::isfinite(Vc0_.real()) && std::isfinite(Vc0_.imag());
      if (!(omega0_ > RealT{0.0}) || !std::isfinite(omega0_) || !finite_phasors)
      {
        Log::error() << "EMT::Bus: invalid angular frequency or initial phasor\n";
        return 1;
      }
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::initialize()
    {
      auto* y  = y_.getData();
      auto* yp = yp_.getData();
      Detail::initializeSinusoid<ScalarT>(Va0_, Vb0_, Vc0_, omega0_, y, yp);
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::tagDifferentiable()
    {
      tag_[0] = true;
      tag_[1] = true;
      tag_[2] = true;
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
