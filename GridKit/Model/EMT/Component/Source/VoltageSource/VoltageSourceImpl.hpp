#pragma once

#include <cmath>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    VoltageSource<scalar_type, index_type>::VoltageSource(BusT*             bus,
                                                          const ModelDataT& data)
      : bus_(bus),
        monitor_(std::make_unique<MonitorT>(data))
    {
      size_ = 6;
      initializeParameters(data);
      initializeMonitor();
    }

    template <typename scalar_type, typename index_type>
    VoltageSource<scalar_type, index_type>::~VoltageSource() = default;

    template <typename scalar_type, typename index_type>
    void VoltageSource<scalar_type, index_type>::initializeParameters(const ModelDataT& data)
    {
      using Parameter = typename ModelDataT::Parameters;
      using Submodel  = typename ModelDataT::Submodels;
      if (data.parameters.contains(Parameter::N))
      {
        N_ = std::get<IdxT>(data.parameters.at(Parameter::N));
      }
      if (data.parameters.contains(Parameter::E))
      {
        E_ = std::get<ABCVector<RealT>>(data.parameters.at(Parameter::E));
      }
      if (data.parameters.contains(Parameter::phi))
      {
        phi_ = std::get<ABCVector<RealT>>(data.parameters.at(Parameter::phi));
      }
      if (data.parameters.contains(Parameter::omega))
      {
        omega_ = std::get<RealT>(data.parameters.at(Parameter::omega));
      }
      if (data.submodels.contains(Submodel::Z))
      {
        const auto z = rationalCoefficients(data.submodels.at(Submodel::Z));
        Rs_          = z.D;
        Ls_          = z.E;
        z_dynamic_   = z.dynamic;
      }
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::allocate()
    {
      if (!allocated_)
      {
        this->allocateVectors(size_);
      }
      tag_.resize(6);
      variable_indices_.resize(6);
      residual_indices_.resize(6);
      for (IdxT index = 0; index < 6; ++index)
      {
        this->setVariableIndex(index, index);
        this->setResidualIndex(index, index);
      }
      wb_.resize(3);
      h_.resize(3);
      allocated_ = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::verify() const
    {
      int status = 0;
      if (bus_ == nullptr)
      {
        Log::error() << "EMT::VoltageSource: bus is null\n";
        ++status;
      }
      if (N_ != IdxT{3})
      {
        Log::error() << "EMT::VoltageSource: only three phases are supported\n";
        ++status;
      }
      if (z_dynamic_)
      {
        Log::error() << "EMT::VoltageSource: Z rational dynamics are not yet "
                        "supported; poles and residues must be empty\n";
        ++status;
      }
      if (!Detail::finite(E_) || E_[0] < RealT{0.0} || E_[1] < RealT{0.0} || E_[2] < RealT{0.0})
      {
        Log::error() << "EMT::VoltageSource: E must be finite and nonnegative\n";
        ++status;
      }
      if (!Detail::finite(phi_))
      {
        Log::error() << "EMT::VoltageSource: phi must be finite\n";
        ++status;
      }
      if (!(omega_ > RealT{0.0}) || !std::isfinite(omega_))
      {
        Log::error() << "EMT::VoltageSource: omega must be finite and positive\n";
        ++status;
      }
      if (!Detail::positiveSemidefinite(Rs_))
      {
        Log::error() << "EMT::VoltageSource: Rs must be finite, symmetric, and positive semidefinite\n";
        ++status;
      }
      if (!Detail::positiveDefinite(Ls_))
      {
        Log::error() << "EMT::VoltageSource: Ls must be finite, symmetric, and positive definite\n";
        ++status;
      }
      return status;
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::initialize()
    {
      std::fill_n(y_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      std::fill_n(yp_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void VoltageSource<scalar_type, index_type>::appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const
    {
      variables.push_back({"i", std::nullopt, 0});
    }

    template <typename scalar_type, typename index_type>
    void VoltageSource<scalar_type, index_type>::appendBusVoltageContributions(
        std::vector<BusVoltageContribution<ScalarT, IdxT>>& contributions) const
    {
      if (bus_ != nullptr)
      {
        contributions.push_back({bus_->busID(), {}, {}});
      }
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::tagDifferentiable()
    {
      tag_[0] = true;
      tag_[1] = true;
      tag_[2] = true;
      tag_[3] = false;
      tag_[4] = false;
      tag_[5] = false;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::setAbsoluteTolerance(RealT absolute_tolerance)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(absolute_tolerance));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int VoltageSource<scalar_type, index_type>::evaluateInternalResidual(
        const ScalarT* y,
        const ScalarT* yp,
        const ScalarT* wb,
        ScalarT*       f)
    {
      const ScalarT ia       = y[0];
      const ScalarT ib       = y[1];
      const ScalarT ic       = y[2];
      const ScalarT ea       = y[3];
      const ScalarT eb       = y[4];
      const ScalarT ec       = y[5];
      const ScalarT iap      = yp[0];
      const ScalarT ibp      = yp[1];
      const ScalarT icp      = yp[2];
      const ScalarT va       = wb[0];
      const ScalarT vb       = wb[1];
      const ScalarT vc       = wb[2];
      const RealT   sqrt_two = std::sqrt(RealT{2.0});

      f[0] = Rs_[0][0] * ia + Rs_[0][1] * ib + Rs_[0][2] * ic
             + Ls_[0][0] * iap + Ls_[0][1] * ibp + Ls_[0][2] * icp + va - ea;
      f[1] = Rs_[1][0] * ia + Rs_[1][1] * ib + Rs_[1][2] * ic
             + Ls_[1][0] * iap + Ls_[1][1] * ibp + Ls_[1][2] * icp + vb - eb;
      f[2] = Rs_[2][0] * ia + Rs_[2][1] * ib + Rs_[2][2] * ic
             + Ls_[2][0] * iap + Ls_[2][1] * ibp + Ls_[2][2] * icp + vc - ec;

      f[3] = ea - sqrt_two * E_[0] * std::cos(omega_ * time_ + phi_[0]);
      f[4] = eb - sqrt_two * E_[1] * std::cos(omega_ * time_ + phi_[1]);
      f[5] = ec - sqrt_two * E_[2] * std::cos(omega_ * time_ + phi_[2]);
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int VoltageSource<scalar_type, index_type>::evaluateBusResidual(
        const ScalarT* y,
        ScalarT*       h)
    {
      h[0] = y[0];
      h[1] = y[1];
      h[2] = y[2];
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::evaluateResidual()
    {
      wb_[0] = bus_->Va();
      wb_[1] = bus_->Vb();
      wb_[2] = bus_->Vc();

      const auto* y  = y_.getData();
      const auto* yp = yp_.getData();
      auto*       f  = f_.getData();
      evaluateInternalResidual(y, yp, wb_.data(), f);
      evaluateBusResidual(bus_->differentiatedKCL() ? yp : y, h_.data());

      bus_->accumulateCurrent(h_[0], h_[1], h_[2]);
      f_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    const Model::VariableMonitorBase* VoltageSource<scalar_type, index_type>::getMonitor() const
    {
      return monitor_.get();
    }

    template <typename scalar_type, typename index_type>
    void VoltageSource<scalar_type, index_type>::initializeMonitor()
    {
      using Variable = typename ModelDataT::MonitorableVariables;
      monitor_->set(Variable::ea, [this]
                    { return y_.getData()[3]; });
      monitor_->set(Variable::eb, [this]
                    { return y_.getData()[4]; });
      monitor_->set(Variable::ec, [this]
                    { return y_.getData()[5]; });
      monitor_->set(Variable::ia, [this]
                    { return y_.getData()[0]; });
      monitor_->set(Variable::ib, [this]
                    { return y_.getData()[1]; });
      monitor_->set(Variable::ic, [this]
                    { return y_.getData()[2]; });
    }
  } // namespace EMT
} // namespace GridKit
