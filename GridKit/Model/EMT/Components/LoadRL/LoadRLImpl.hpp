#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <variant>

#include <GridKit/Model/EMT/Components/LoadRL/LoadRL.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    LoadRL<ScalarT, IdxT>::LoadRL(BusT* bus, const DataT& data)
      : bus_(bus),
        data_(data)
    {
      size_ = variable_count;
      if (bus_ != nullptr && !data_.ports.contains(DataT::Ports::ac))
      {
        data_.ports[DataT::Ports::ac] = bus_->busID();
      }
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    LoadRL<ScalarT, IdxT>::LoadRL(const DataT& data)
      : data_(data)
    {
      size_ = variable_count;
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    LoadRL<ScalarT, IdxT>::~LoadRL()
    {
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::allocate()
    {
      size_             = variable_count;
      const size_t size = static_cast<size_t>(size_);
      y_.resize(size);
      yp_.resize(size);
      f_.resize(size);
      tag_.resize(size);
      variable_indices_.resize(size);
      residual_indices_.resize(size);

      for (IdxT j = 0; j < size_; ++j)
      {
        this->setVariableIndex(j, j);
        this->setResidualIndex(j, j);
      }

      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::initialize()
    {
      std::fill(y_.begin(), y_.end(), ScalarT{0.0});
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      if (bus_ == nullptr)
      {
        return 0;
      }

      const auto                voltage = bus_->initialVoltagePhasor();
      const auto                omega   = bus_->omega();
      const RealT               sqrt2   = std::sqrt(RealT{2.0});
      const std::complex<RealT> j{0.0, 1.0};
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        const std::complex<RealT> impedance{r_[phase], omega * l_[phase]};
        const auto                current = -voltage[phase] / impedance;
        y_[phase]                         = static_cast<ScalarT>(sqrt2 * std::real(current));
        yp_[phase]                        = static_cast<ScalarT>(sqrt2 * std::real(j * omega * current));
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::evaluateResidual()
    {
      // Residual assembly is system-driven via SystemModel + LocalMap; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::evaluateJacobian()
    {
      // Jacobian assembly is system-driven via SystemModel + SparseAD; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::verify() const
    {
      int errors  = 0;
      errors     += data_.ports.contains(DataT::Ports::ac) ? 0 : 1;

      const auto r_iter = data_.parameters.find(DataT::Parameters::r);
      const auto l_iter = data_.parameters.find(DataT::Parameters::l);
      if (r_iter == data_.parameters.end()
          || std::get_if<PhaseVector<RealT>>(&r_iter->second) == nullptr)
      {
        ++errors;
      }
      if (l_iter == data_.parameters.end()
          || std::get_if<PhaseVector<RealT>>(&l_iter->second) == nullptr)
      {
        ++errors;
      }

      if (errors != 0)
      {
        return errors;
      }

      const auto& r = *std::get_if<PhaseVector<RealT>>(&r_iter->second);
      const auto& l = *std::get_if<PhaseVector<RealT>>(&l_iter->second);
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        if (!std::isfinite(r[phase]) || r[phase] < RealT{0.0})
        {
          ++errors;
        }
        if (!std::isfinite(l[phase]) || l[phase] <= RealT{0.0})
        {
          ++errors;
        }
      }

      return errors;
    }

    template <class ScalarT, typename IdxT>
    const Model::VariableMonitorBase* LoadRL<ScalarT, IdxT>::getMonitor() const
    {
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    typename LoadRL<ScalarT, IdxT>::BusT* LoadRL<ScalarT, IdxT>::connectedBus(std::size_t port) const
    {
      if (port != 0)
      {
        throw std::out_of_range("LoadRL has one port");
      }
      return bus_;
    }

    template <class ScalarT, typename IdxT>
    const typename LoadRL<ScalarT, IdxT>::DataT& LoadRL<ScalarT, IdxT>::data() const
    {
      return data_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseVector<typename LoadRL<ScalarT, IdxT>::RealT>& LoadRL<ScalarT, IdxT>::r() const
    {
      return r_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseVector<typename LoadRL<ScalarT, IdxT>::RealT>& LoadRL<ScalarT, IdxT>::l() const
    {
      return l_;
    }

    template <class ScalarT, typename IdxT>
    void LoadRL<ScalarT, IdxT>::loadParameters()
    {
      if (const auto iter = data_.parameters.find(DataT::Parameters::r);
          iter != data_.parameters.end())
      {
        if (const auto* r = std::get_if<PhaseVector<RealT>>(&iter->second))
        {
          r_ = *r;
        }
      }

      if (const auto iter = data_.parameters.find(DataT::Parameters::l);
          iter != data_.parameters.end())
      {
        if (const auto* l = std::get_if<PhaseVector<RealT>>(&iter->second))
        {
          l_ = *l;
        }
      }
    }
  } // namespace EMT
} // namespace GridKit
