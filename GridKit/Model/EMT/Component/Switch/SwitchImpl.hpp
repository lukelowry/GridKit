#pragma once

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Component/Switch/Switch.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    Switch<scalar_type, index_type>::Switch(BusT*             bus1,
                                            BusT*             bus2,
                                            const ModelDataT& data)
      : bus1_(bus1),
        bus2_(bus2),
        monitor_(std::make_unique<MonitorT>(data))
    {
      size_ = 3;
      initializeParameters(data);
      initializeMonitor();
    }

    template <typename scalar_type, typename index_type>
    Switch<scalar_type, index_type>::~Switch() = default;

    template <typename scalar_type, typename index_type>
    void Switch<scalar_type, index_type>::initializeParameters(const ModelDataT& data)
    {
      using Parameter = typename ModelDataT::Parameters;
      if (data.parameters.contains(Parameter::N))
      {
        N_ = std::get<IdxT>(data.parameters.at(Parameter::N));
      }
    }

    template <typename scalar_type, typename index_type>
    auto Switch<scalar_type, index_type>::openCommand() const -> RealT
    {
      return Detail::scalarValue<RealT>(
          signals_.template readExternalVariable<SwitchExternalVariables::open>());
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::allocate()
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
    int Switch<scalar_type, index_type>::verify() const
    {
      int status = 0;
      if (bus1_ == nullptr || bus2_ == nullptr || bus1_ == bus2_)
      {
        Log::error() << "EMT::Switch: two distinct, non-null buses are required\n";
        ++status;
      }
      if (N_ != IdxT{3})
      {
        Log::error() << "EMT::Switch: only three phases are supported\n";
        ++status;
      }

      static constexpr auto open = SwitchExternalVariables::open;
      if (!signals_.template isAttached<open>())
      {
        Log::error() << "EMT::Switch: open signal is not attached\n";
        ++status;
      }
      else if (!signals_.template isLinked<open>())
      {
        Log::error() << "EMT::Switch: open signal is attached but not linked\n";
        ++status;
      }
      else if (openCommand() != RealT{0.0} && openCommand() != RealT{1.0})
      {
        Log::error() << "EMT::Switch: open signal must be exactly 0 or 1\n";
        ++status;
      }
      return status;
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::initialize()
    {
      std::fill_n(y_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      std::fill_n(yp_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void Switch<scalar_type, index_type>::appendBusVoltageContributions(
        std::vector<BusVoltageContribution<ScalarT, IdxT>>& contributions) const
    {
      // The switch stores no energy and injects an algebraic current, so it
      // contributes no bus-voltage coefficient and cannot support a
      // differentiated current balance at either terminal.
      if (bus1_ != nullptr)
      {
        contributions.push_back({bus1_->busID(), {}, {}, false});
      }
      if (bus2_ != nullptr)
      {
        contributions.push_back({bus2_->busID(), {}, {}, false});
      }
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::tagDifferentiable()
    {
      tag_[0] = false;
      tag_[1] = false;
      tag_[2] = false;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::setAbsoluteTolerance(RealT absolute_tolerance)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(absolute_tolerance));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::evaluateResidual()
    {
      const RealT open   = openCommand();
      const RealT closed = RealT{1.0} - open;

      const auto* y = y_.getData();
      auto*       f = f_.getData();
      f[0]          = open * y[0] + closed * (bus2_->Va() - bus1_->Va());
      f[1]          = open * y[1] + closed * (bus2_->Vb() - bus1_->Vb());
      f[2]          = open * y[2] + closed * (bus2_->Vc() - bus1_->Vc());

      bus1_->accumulateCurrent(-y[0], -y[1], -y[2]);
      bus2_->accumulateCurrent(y[0], y[1], y[2]);
      f_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    auto Switch<scalar_type, index_type>::getSignals() -> SignalsT&
    {
      return signals_;
    }

    template <typename scalar_type, typename index_type>
    const Model::VariableMonitorBase* Switch<scalar_type, index_type>::getMonitor() const
    {
      return monitor_.get();
    }

    template <typename scalar_type, typename index_type>
    void Switch<scalar_type, index_type>::initializeMonitor()
    {
      using Variable = typename ModelDataT::MonitorableVariables;
      monitor_->set(Variable::open, [this]
                    { return ScalarT{openCommand()}; });
      monitor_->set(Variable::i12a, [this]
                    { return y_.getData()[0]; });
      monitor_->set(Variable::i12b, [this]
                    { return y_.getData()[1]; });
      monitor_->set(Variable::i12c, [this]
                    { return y_.getData()[2]; });
    }
  } // namespace EMT
} // namespace GridKit
