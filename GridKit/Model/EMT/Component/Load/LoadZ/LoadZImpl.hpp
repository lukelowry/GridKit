#pragma once

#include <cmath>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadZ/LoadZ.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    LoadZ<scalar_type, index_type>::LoadZ(BusT*             bus,
                                          const ModelDataT& data)
      : bus_(bus),
        monitor_(std::make_unique<MonitorT>(data))
    {
      size_ = 3;
      initializeParameters(data);
      initializeMonitor();
    }

    template <typename scalar_type, typename index_type>
    LoadZ<scalar_type, index_type>::~LoadZ() = default;

    template <typename scalar_type, typename index_type>
    void LoadZ<scalar_type, index_type>::initializeParameters(const ModelDataT& data)
    {
      using Parameter = typename ModelDataT::Parameters;
      using Submodel  = typename ModelDataT::Submodels;
      if (data.parameters.contains(Parameter::N))
      {
        N_ = std::get<IdxT>(data.parameters.at(Parameter::N));
      }
      if (data.submodels.contains(Submodel::Z))
      {
        const auto z = rationalCoefficients(data.submodels.at(Submodel::Z));
        R_           = z.D;
        L_           = z.E;
        z_dynamic_   = z.dynamic;
      }
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::allocate()
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
      wb_.resize(3);
      h_.resize(3);
      allocated_ = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::verify() const
    {
      int status = 0;
      if (bus_ == nullptr)
      {
        Log::error() << "EMT::LoadZ: bus is null\n";
        ++status;
      }
      if (N_ != IdxT{3})
      {
        Log::error() << "EMT::LoadZ: only three phases are supported\n";
        ++status;
      }
      if (z_dynamic_)
      {
        Log::error() << "EMT::LoadZ: Z rational dynamics are not yet "
                        "supported; poles and residues must be empty\n";
        ++status;
      }
      if (!Detail::positiveSemidefinite(R_))
      {
        Log::error() << "EMT::LoadZ: R must be finite, symmetric, and positive semidefinite\n";
        ++status;
      }
      if (!Detail::positiveDefinite(L_))
      {
        Log::error() << "EMT::LoadZ: L must be finite, symmetric, and positive definite\n";
        ++status;
      }

      static constexpr auto enable = LoadZExternalVariables::enable;
      if (!signals_.template isAttached<enable>())
      {
        Log::error() << "EMT::LoadZ: enable signal is not attached\n";
        ++status;
      }
      else if (!signals_.template isLinked<enable>())
      {
        Log::error() << "EMT::LoadZ: enable signal is attached but not linked\n";
        ++status;
      }
      return status;
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::initialize()
    {
      std::fill_n(y_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      std::fill_n(yp_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void LoadZ<scalar_type, index_type>::appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const
    {
      variables.push_back({"i", std::nullopt, 0});
    }

    template <typename scalar_type, typename index_type>
    void LoadZ<scalar_type, index_type>::appendBusVoltageContributions(
        std::vector<BusVoltageContribution<ScalarT, IdxT>>& contributions) const
    {
      if (bus_ != nullptr)
      {
        contributions.push_back({bus_->busID(), {}, {}});
      }
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::tagDifferentiable()
    {
      tag_[0] = true;
      tag_[1] = true;
      tag_[2] = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::setAbsoluteTolerance(RealT absolute_tolerance)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(absolute_tolerance));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int LoadZ<scalar_type, index_type>::evaluateInternalResidual(
        const ScalarT* y,
        const ScalarT* yp,
        const ScalarT* wb,
        ScalarT*       f)
    {
      const ScalarT ia  = y[0];
      const ScalarT ib  = y[1];
      const ScalarT ic  = y[2];
      const ScalarT iap = yp[0];
      const ScalarT ibp = yp[1];
      const ScalarT icp = yp[2];
      const ScalarT va  = wb[0];
      const ScalarT vb  = wb[1];
      const ScalarT vc  = wb[2];

      f[0] = R_[0][0] * ia + R_[0][1] * ib + R_[0][2] * ic
             + L_[0][0] * iap + L_[0][1] * ibp + L_[0][2] * icp + va;
      f[1] = R_[1][0] * ia + R_[1][1] * ib + R_[1][2] * ic
             + L_[1][0] * iap + L_[1][1] * ibp + L_[1][2] * icp + vb;
      f[2] = R_[2][0] * ia + R_[2][1] * ib + R_[2][2] * ic
             + L_[2][0] * iap + L_[2][1] * ibp + L_[2][2] * icp + vc;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int LoadZ<scalar_type, index_type>::evaluateBusResidual(
        const ScalarT* y,
        ScalarT        enable,
        ScalarT*       h)
    {
      h[0] = enable * y[0];
      h[1] = enable * y[1];
      h[2] = enable * y[2];
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::evaluateResidual()
    {
      wb_[0] = bus_->Va();
      wb_[1] = bus_->Vb();
      wb_[2] = bus_->Vc();

      ScalarT enable{1.0};
      if (signals_.template isAttached<LoadZExternalVariables::enable>())
      {
        enable = signals_.template readExternalVariable<LoadZExternalVariables::enable>();
      }

      const auto* y  = y_.getData();
      const auto* yp = yp_.getData();
      auto*       f  = f_.getData();
      evaluateInternalResidual(y, yp, wb_.data(), f);
      evaluateBusResidual(bus_->differentiatedKCL() ? yp : y,
                          enable,
                          h_.data());

      bus_->accumulateCurrent(h_[0], h_[1], h_[2]);
      f_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    auto LoadZ<scalar_type, index_type>::getSignals() -> SignalsT&
    {
      return signals_;
    }

    template <typename scalar_type, typename index_type>
    const Model::VariableMonitorBase* LoadZ<scalar_type, index_type>::getMonitor() const
    {
      return monitor_.get();
    }

    template <typename scalar_type, typename index_type>
    void LoadZ<scalar_type, index_type>::initializeMonitor()
    {
      using Variable = typename ModelDataT::MonitorableVariables;
      monitor_->set(Variable::ia, [this]
                    { return y_.getData()[0]; });
      monitor_->set(Variable::ib, [this]
                    { return y_.getData()[1]; });
      monitor_->set(Variable::ic, [this]
                    { return y_.getData()[2]; });
    }
  } // namespace EMT
} // namespace GridKit
