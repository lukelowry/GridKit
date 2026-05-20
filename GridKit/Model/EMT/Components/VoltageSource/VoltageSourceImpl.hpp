#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

#include <GridKit/Model/EMT/Components/VoltageSource/VoltageSource.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::VoltageSource(BusT* bus, const DataT& data)
      : bus_(bus),
        data_(data)
    {
      size_ = variable_count;
      if (bus_ != nullptr && !data_.ports.contains(DataT::Ports::bus))
      {
        data_.ports[DataT::Ports::bus] = bus_->busID();
      }
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::VoltageSource(const DataT& data)
      : data_(data)
    {
      size_ = variable_count;
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>::~VoltageSource()
    {
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::allocate()
    {
      size_ = variable_count;
      y_.clear();
      yp_.clear();
      f_.clear();
      tag_.clear();
      variable_indices_.clear();
      residual_indices_.clear();
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::initialize()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::tagDifferentiable()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateResidual()
    {
      // Residual assembly is system-driven via SystemModel + LocalMap; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateJacobian()
    {
      // Jacobian assembly is system-driven via SystemModel + SparseAD; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::verify() const
    {
      int errors  = 0;
      errors     += data_.ports.contains(DataT::Ports::bus) ? 0 : 1;

      const auto e_iter     = data_.parameters.find(DataT::Parameters::e);
      const auto phi_iter   = data_.parameters.find(DataT::Parameters::phi);
      const auto r_iter     = data_.parameters.find(DataT::Parameters::r);
      const auto freq_iter  = data_.parameters.find(DataT::Parameters::frequency);
      const auto omega_iter = data_.parameters.find(DataT::Parameters::omega0);

      if (e_iter == data_.parameters.end()
          || std::get_if<PhaseVector<RealT>>(&e_iter->second) == nullptr)
      {
        ++errors;
      }
      if (phi_iter == data_.parameters.end()
          || std::get_if<PhaseVector<RealT>>(&phi_iter->second) == nullptr)
      {
        ++errors;
      }
      if (r_iter == data_.parameters.end()
          || std::get_if<PhaseVector<RealT>>(&r_iter->second) == nullptr)
      {
        ++errors;
      }
      if (freq_iter == data_.parameters.end() && omega_iter == data_.parameters.end())
      {
        ++errors;
      }

      if (errors != 0)
      {
        return errors;
      }

      const auto& e = *std::get_if<PhaseVector<RealT>>(&e_iter->second);
      const auto& r = *std::get_if<PhaseVector<RealT>>(&r_iter->second);
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        if (!std::isfinite(e[phase]) || e[phase] < RealT{0.0})
        {
          ++errors;
        }
        if (!std::isfinite(r[phase]) || r[phase] <= RealT{0.0})
        {
          ++errors;
        }
      }

      auto realValue = [](const typename DataT::ParameterValue& value, RealT& out)
      {
        if (const auto* real = std::get_if<RealT>(&value))
        {
          out = *real;
          return true;
        }
        return false;
      };

      const RealT pi = std::acos(RealT{-1.0});
      RealT       frequency{};
      RealT       omega{};
      const bool  has_frequency =
          freq_iter != data_.parameters.end() && realValue(freq_iter->second, frequency);
      const bool has_omega =
          omega_iter != data_.parameters.end() && realValue(omega_iter->second, omega);

      if (freq_iter != data_.parameters.end() && !has_frequency)
      {
        ++errors;
      }
      if (omega_iter != data_.parameters.end() && !has_omega)
      {
        ++errors;
      }
      if (has_frequency && (!std::isfinite(frequency) || frequency <= RealT{0.0}))
      {
        ++errors;
      }
      if (has_omega && (!std::isfinite(omega) || omega <= RealT{0.0}))
      {
        ++errors;
      }
      if (has_frequency && has_omega)
      {
        const RealT converted = RealT{2.0} * pi * frequency;
        const RealT error     = std::abs(converted - omega) / (RealT{1.0} + std::abs(omega));
        if (error > RealT{1.0e-10})
        {
          ++errors;
        }
      }

      return errors;
    }

    template <class ScalarT, typename IdxT>
    const Model::VariableMonitorBase* VoltageSource<ScalarT, IdxT>::getMonitor() const
    {
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    typename VoltageSource<ScalarT, IdxT>::BusT*
    VoltageSource<ScalarT, IdxT>::connectedBus(std::size_t port) const
    {
      if (port != 0)
      {
        throw std::out_of_range("VoltageSource has one port");
      }
      return bus_;
    }

    template <class ScalarT, typename IdxT>
    const typename VoltageSource<ScalarT, IdxT>::DataT& VoltageSource<ScalarT, IdxT>::data() const
    {
      return data_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseVector<typename VoltageSource<ScalarT, IdxT>::RealT>&
    VoltageSource<ScalarT, IdxT>::e() const
    {
      return e_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseVector<typename VoltageSource<ScalarT, IdxT>::RealT>&
    VoltageSource<ScalarT, IdxT>::phi() const
    {
      return phi_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseVector<typename VoltageSource<ScalarT, IdxT>::RealT>&
    VoltageSource<ScalarT, IdxT>::r() const
    {
      return r_;
    }

    template <class ScalarT, typename IdxT>
    typename VoltageSource<ScalarT, IdxT>::RealT VoltageSource<ScalarT, IdxT>::omega0() const
    {
      return omega0_;
    }

    template <class ScalarT, typename IdxT>
    void VoltageSource<ScalarT, IdxT>::loadParameters()
    {
      if (const auto iter = data_.parameters.find(DataT::Parameters::e);
          iter != data_.parameters.end())
      {
        if (const auto* e = std::get_if<PhaseVector<RealT>>(&iter->second))
        {
          e_ = *e;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::phi);
          iter != data_.parameters.end())
      {
        if (const auto* phi = std::get_if<PhaseVector<RealT>>(&iter->second))
        {
          phi_ = *phi;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::r);
          iter != data_.parameters.end())
      {
        if (const auto* r = std::get_if<PhaseVector<RealT>>(&iter->second))
        {
          r_ = *r;
        }
      }

      const RealT pi = std::acos(RealT{-1.0});
      if (const auto iter = data_.parameters.find(DataT::Parameters::omega0);
          iter != data_.parameters.end())
      {
        if (const auto* omega = std::get_if<RealT>(&iter->second))
        {
          omega0_ = *omega;
          return;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::frequency);
          iter != data_.parameters.end())
      {
        if (const auto* frequency = std::get_if<RealT>(&iter->second))
        {
          omega0_ = RealT{2.0} * pi * *frequency;
        }
      }
    }
  } // namespace EMT
} // namespace GridKit
