#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <variant>

#include <GridKit/Model/EMT/Components/BranchLumpedConstant/BranchLumpedConstant.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    BranchLumpedConstant<ScalarT, IdxT>::BranchLumpedConstant(BusT*        from_bus,
                                                              BusT*        to_bus,
                                                              const DataT& data)
      : from_bus_(from_bus),
        to_bus_(to_bus),
        data_(data)
    {
      size_ = variable_count;
      if (from_bus_ != nullptr && !data_.ports.contains(DataT::Ports::from))
      {
        data_.ports[DataT::Ports::from] = from_bus_->busID();
      }
      if (to_bus_ != nullptr && !data_.ports.contains(DataT::Ports::to))
      {
        data_.ports[DataT::Ports::to] = to_bus_->busID();
      }
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    BranchLumpedConstant<ScalarT, IdxT>::BranchLumpedConstant(const DataT& data)
      : data_(data)
    {
      size_ = variable_count;
      loadParameters();
    }

    template <class ScalarT, typename IdxT>
    BranchLumpedConstant<ScalarT, IdxT>::~BranchLumpedConstant()
    {
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::setGridKitComponentID(IdxT id)
    {
      gridkit_component_id_ = id;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::allocate()
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
    int BranchLumpedConstant<ScalarT, IdxT>::initialize()
    {
      std::fill(y_.begin(), y_.end(), ScalarT{0.0});
      std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
      if (from_bus_ == nullptr || to_bus_ == nullptr)
      {
        return 0;
      }

      const auto  from_voltage      = from_bus_->initialVoltagePhasor();
      const auto  to_voltage        = to_bus_->initialVoltagePhasor();
      const auto  from_omega        = from_bus_->omega();
      const auto  to_omega          = to_bus_->omega();
      const RealT scale_denominator = RealT{1.0} + std::max(std::abs(from_omega), std::abs(to_omega));
      if (std::abs(from_omega - to_omega) / scale_denominator > RealT{1.0e-12})
      {
        throw std::invalid_argument("BranchLumpedConstant endpoint buses must have matching initial omega");
      }

      using Complex = std::complex<RealT>;
      PhaseMatrix<Complex> impedance{};
      PhaseVector<Complex> voltage_drop{};
      const Complex        j{0.0, 1.0};
      for (std::size_t row = 0; row < 3; ++row)
      {
        voltage_drop[row] = from_voltage[row] - to_voltage[row];
        for (std::size_t col = 0; col < 3; ++col)
        {
          impedance[row][col] = Complex{R_[row][col], from_omega * L_[row][col]};
        }
      }

      const auto  current = solve(impedance, voltage_drop);
      const RealT sqrt2   = std::sqrt(RealT{2.0});
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        y_[phase]  = static_cast<ScalarT>(sqrt2 * std::real(current[phase]));
        yp_[phase] = static_cast<ScalarT>(sqrt2 * std::real(j * from_omega * current[phase]));
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::evaluateResidual()
    {
      // Residual assembly is system-driven via SystemModel + LocalMap; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::evaluateJacobian()
    {
      // Jacobian assembly is system-driven via SystemModel + SparseAD; this Evaluator override is intentionally unused.
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::verify() const
    {
      int errors  = 0;
      errors     += data_.ports.contains(DataT::Ports::from) ? 0 : 1;
      errors     += data_.ports.contains(DataT::Ports::to) ? 0 : 1;

      const auto r_iter      = data_.parameters.find(DataT::Parameters::r);
      const auto l_iter      = data_.parameters.find(DataT::Parameters::l);
      const auto g_iter      = data_.parameters.find(DataT::Parameters::g);
      const auto c_iter      = data_.parameters.find(DataT::Parameters::c);
      const auto length_iter = data_.parameters.find(DataT::Parameters::length);

      if (r_iter == data_.parameters.end()
          || std::get_if<PhaseMatrix<RealT>>(&r_iter->second) == nullptr)
      {
        ++errors;
      }
      if (l_iter == data_.parameters.end()
          || std::get_if<PhaseMatrix<RealT>>(&l_iter->second) == nullptr)
      {
        ++errors;
      }
      if (g_iter != data_.parameters.end()
          && std::get_if<PhaseMatrix<RealT>>(&g_iter->second) == nullptr)
      {
        ++errors;
      }
      if (c_iter == data_.parameters.end()
          || std::get_if<PhaseMatrix<RealT>>(&c_iter->second) == nullptr)
      {
        ++errors;
      }
      if (length_iter == data_.parameters.end()
          || std::get_if<RealT>(&length_iter->second) == nullptr)
      {
        ++errors;
      }

      if (errors != 0)
      {
        return errors;
      }

      const auto& r      = *std::get_if<PhaseMatrix<RealT>>(&r_iter->second);
      const auto& l      = *std::get_if<PhaseMatrix<RealT>>(&l_iter->second);
      const auto& c      = *std::get_if<PhaseMatrix<RealT>>(&c_iter->second);
      const auto  length = *std::get_if<RealT>(&length_iter->second);
      if (!isFinite(r) || !isFinite(l) || !isFinite(c))
      {
        ++errors;
      }
      if (g_iter != data_.parameters.end())
      {
        const auto& g = *std::get_if<PhaseMatrix<RealT>>(&g_iter->second);
        if (!isFinite(g))
        {
          ++errors;
        }
      }
      if (!std::isfinite(length) || length <= RealT{0.0})
      {
        ++errors;
      }

      for (std::size_t row = 0; row < 3; ++row)
      {
        if (c[row][row] <= RealT{0.0})
        {
          ++errors;
        }
        for (std::size_t col = row + 1; col < 3; ++col)
        {
          const RealT scale = RealT{1.0} + std::max(std::abs(c[row][col]), std::abs(c[col][row]));
          if (std::abs(c[row][col] - c[col][row]) / scale > RealT{1.0e-12})
          {
            ++errors;
          }
        }
      }

      if (std::isfinite(length) && length > RealT{0.0} && isFinite(l))
      {
        const auto derived_l = scale(l, length);
        if (isSingular(derived_l))
        {
          ++errors;
        }
      }

      return errors;
    }

    template <class ScalarT, typename IdxT>
    const Model::VariableMonitorBase* BranchLumpedConstant<ScalarT, IdxT>::getMonitor() const
    {
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    typename BranchLumpedConstant<ScalarT, IdxT>::BusT*
    BranchLumpedConstant<ScalarT, IdxT>::connectedBus(std::size_t port) const
    {
      if (port == 0)
      {
        return from_bus_;
      }
      if (port == 1)
      {
        return to_bus_;
      }
      throw std::out_of_range("BranchLumpedConstant has two ports");
    }

    template <class ScalarT, typename IdxT>
    const typename BranchLumpedConstant<ScalarT, IdxT>::DataT&
    BranchLumpedConstant<ScalarT, IdxT>::data() const
    {
      return data_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseMatrix<typename BranchLumpedConstant<ScalarT, IdxT>::RealT>&
    BranchLumpedConstant<ScalarT, IdxT>::R() const
    {
      return R_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseMatrix<typename BranchLumpedConstant<ScalarT, IdxT>::RealT>&
    BranchLumpedConstant<ScalarT, IdxT>::L() const
    {
      return L_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseMatrix<typename BranchLumpedConstant<ScalarT, IdxT>::RealT>&
    BranchLumpedConstant<ScalarT, IdxT>::GHalf() const
    {
      return G_half_;
    }

    template <class ScalarT, typename IdxT>
    const PhaseMatrix<typename BranchLumpedConstant<ScalarT, IdxT>::RealT>&
    BranchLumpedConstant<ScalarT, IdxT>::CHalf() const
    {
      return C_half_;
    }

    template <class ScalarT, typename IdxT>
    void BranchLumpedConstant<ScalarT, IdxT>::loadParameters()
    {
      if (const auto iter = data_.parameters.find(DataT::Parameters::r);
          iter != data_.parameters.end())
      {
        if (const auto* r = std::get_if<PhaseMatrix<RealT>>(&iter->second))
        {
          r_per_m_ = *r;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::l);
          iter != data_.parameters.end())
      {
        if (const auto* l = std::get_if<PhaseMatrix<RealT>>(&iter->second))
        {
          l_per_m_ = *l;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::g);
          iter != data_.parameters.end())
      {
        if (const auto* g = std::get_if<PhaseMatrix<RealT>>(&iter->second))
        {
          g_per_m_ = *g;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::c);
          iter != data_.parameters.end())
      {
        if (const auto* c = std::get_if<PhaseMatrix<RealT>>(&iter->second))
        {
          c_per_m_ = *c;
        }
      }
      if (const auto iter = data_.parameters.find(DataT::Parameters::length);
          iter != data_.parameters.end())
      {
        if (const auto* length = std::get_if<RealT>(&iter->second))
        {
          length_ = *length;
        }
      }

      R_      = scale(r_per_m_, length_);
      L_      = scale(l_per_m_, length_);
      G_half_ = scale(g_per_m_, RealT{0.5} * length_);
      C_half_ = scale(c_per_m_, RealT{0.5} * length_);
    }
  } // namespace EMT
} // namespace GridKit
