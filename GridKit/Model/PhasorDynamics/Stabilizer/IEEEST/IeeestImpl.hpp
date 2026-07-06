#pragma once

/**
 * @file IeeestImpl.hpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Definition of the IEEEST Power System Stabilizer.
 */

#include <algorithm>
#include <iostream>
#include <variant>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/Ieeest.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/IeeestData.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Stabilizer
    {
      using Log = ::GridKit::Utilities::Logger;

      template <typename scalar_type, typename index_type, size_t order>
      Ieeest<scalar_type, index_type, order>::Ieeest()
      {
        size_ = static_cast<IdxT>(IeeestInternalVariables<order>::MAXIMUM);
        setDerivedParameters();
      }

      template <typename scalar_type, typename index_type, size_t order>
      Ieeest<scalar_type, index_type, order>::Ieeest(const ModelDataT& data)
        : monitor_(std::make_unique<MonitorT>(data))
      {
        initializeParameters(data);
        initializeMonitor();
        size_ = static_cast<IdxT>(IeeestInternalVariables<order>::MAXIMUM);
        setDerivedParameters();
      }

      template <typename scalar_type, typename index_type, size_t order>
      Ieeest<scalar_type, index_type, order>::~Ieeest()
      {
      }

      template <typename scalar_type, typename index_type, size_t order>
      void Ieeest<scalar_type, index_type, order>::setDerivedParameters()
      {
        T2_ = std::max(T2_, TIME_CONSTANT_MINIMUM);
        T4_ = std::max(T4_, TIME_CONSTANT_MINIMUM);
        T6_ = std::max(T6_, TIME_CONSTANT_MINIMUM);

        const auto a = notchCoefficients(A1_, A2_, A3_, A4_);

        a1_ = a[0];
        a2_ = a[1];
        a3_ = a[2];
        a4_ = a[3];
      }

      template <typename scalar_type, typename index_type, size_t order>
      void Ieeest<scalar_type, index_type, order>::initializeParameters(const ModelDataT& data)
      {
        using Params = typename ModelDataT::Parameters;

        parameter_error_count_ = 0;

        auto load_real = [&](auto key, RealT& target, const char* name)
        {
          if (!data.parameters.contains(key))
          {
            return;
          }

          const auto& value = data.parameters.at(key);
          if (const auto* real_value = std::get_if<RealT>(&value))
          {
            target = *real_value;
          }
          else if (const auto* index_value = std::get_if<IdxT>(&value))
          {
            target = static_cast<RealT>(*index_value);
          }
          else
          {
            Log::error() << "Ieeest: parameter '" << name << "' must be numeric\n";
            ++parameter_error_count_;
          }
        };

        load_real(Params::A1, A1_, "A1");
        load_real(Params::A2, A2_, "A2");
        load_real(Params::A3, A3_, "A3");
        load_real(Params::A4, A4_, "A4");
        load_real(Params::A5, A5_, "A5");
        load_real(Params::A6, A6_, "A6");
        load_real(Params::T1, T1_, "T1");
        load_real(Params::T2, T2_, "T2");
        load_real(Params::T3, T3_, "T3");
        load_real(Params::T4, T4_, "T4");
        load_real(Params::T5, T5_, "T5");
        load_real(Params::T6, T6_, "T6");
        load_real(Params::Ks, Ks_, "Ks");
        load_real(Params::Lsmin, Lsmin_, "Lsmin");
        load_real(Params::Lsmax, Lsmax_, "Lsmax");
        load_real(Params::Vcl, Vcl_, "Vcl");
        load_real(Params::Vcu, Vcu_, "Vcu");
        load_real(Params::Tdelay, Tdelay_, "Tdelay");
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::allocate()
      {
        size_     = static_cast<IdxT>(IeeestInternalVariables<order>::MAXIMUM);
        auto size = static_cast<size_t>(size_);

        f_.assign(size, ScalarT{0});
        y_.assign(size, ScalarT{0});
        yp_.assign(size, ScalarT{0});
        tag_.assign(size, false);
        abs_tol_.assign(size, ScalarT{0});
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        auto signal_size = static_cast<size_t>(IeeestExternalVariables::MAXIMUM);
        ws_.assign(signal_size, ScalarT{0});
        ws_indices_.assign(signal_size, INVALID_INDEX<IdxT>);

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        constexpr auto VSS = IeeestInternalVariables<order>::VSS;
        if (signals_.template isAssigned<VSS>())
        {
          signals_.template getSignalNode<VSS>()->set(
              &y_[static_cast<size_t>(VSS)],
              &(this->getVariableIndex(static_cast<IdxT>(VSS))));
        }

        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::verify() const
      {
        int ret = static_cast<int>(parameter_error_count_);

        if (signals_.template isAttached<IeeestExternalVariables::U>())
        {
          if (!signals_.template isLinked<IeeestExternalVariables::U>())
          {
            Log::error() << "Ieeest: input signal U attached with no linked source\n";
            ret += 1;
          }
        }
        else
        {
          Log::error() << "Ieeest: required input signal U is not attached\n";
          ret += 1;
        }

        const size_t derived_order = notchOrder(a1_, a2_, a3_, a4_);
        if (derived_order != order)
        {
          Log::error() << "Ieeest: parameters imply a notch filter of order " << derived_order
                       << " for a model instantiated with order " << order << "\n";
          ret += 1;
        }

        if constexpr (order == 0)
        {
          if (A5_ != ZERO<RealT> || A6_ != ZERO<RealT>)
          {
            Log::error() << "Ieeest: unsupported zeroth-order notch filter with nonzero numerator\n";
            ret += 1;
          }
        }

        if constexpr (order == 1)
        {
          if (A6_ != ZERO<RealT>)
          {
            Log::error() << "Ieeest: unsupported first-order notch filter with second-order numerator\n";
            ret += 1;
          }
        }

        return ret;
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::initialize()
      {
        if (verify() > 0)
        {
          Log::error() << "Ieeest: cannot initialize with invalid configuration\n";
          return 1;
        }

        const auto X5  = static_cast<size_t>(IeeestInternalVariables<order>::X5);
        const auto X6  = static_cast<size_t>(IeeestInternalVariables<order>::X6);
        const auto X7  = static_cast<size_t>(IeeestInternalVariables<order>::X7);
        const auto V4  = static_cast<size_t>(IeeestInternalVariables<order>::V4);
        const auto V5  = static_cast<size_t>(IeeestInternalVariables<order>::V5);
        const auto V6  = static_cast<size_t>(IeeestInternalVariables<order>::V6);
        const auto V7  = static_cast<size_t>(IeeestInternalVariables<order>::V7);
        const auto VSS = static_cast<size_t>(IeeestInternalVariables<order>::VSS);
        const auto U   = static_cast<size_t>(IeeestExternalVariables::U);

        std::fill(y_.begin(), y_.end(), ZERO<RealT>);
        std::fill(yp_.begin(), yp_.end(), ZERO<RealT>);

        const ScalarT u = signals_.template readExternalVariable<IeeestExternalVariables::U>();
        ws_[U]          = u;
        ws_indices_[U]  = signals_.template readExternalVariableIndex<IeeestExternalVariables::U>();

        // Chain states x2..xN hold successive derivatives of the filtered
        // signal and remain at zero from the fill above.
        if constexpr (order >= 1)
        {
          const auto X1 = static_cast<size_t>(IeeestInternalVariables<order>::X1);

          y_[X1] = u;
        }

        y_[X5]  = u;
        y_[X6]  = u;
        y_[X7]  = u;
        y_[V4]  = u;
        y_[V5]  = u;
        y_[V6]  = u;
        y_[V7]  = ZERO<RealT>;
        y_[VSS] = Math::clamp(y_[V7], Lsmin_, Lsmax_);

        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::tagDifferentiable()
      {
        std::fill(tag_.begin(), tag_.end(), false);

        if constexpr (order >= 1)
        {
          tag_[static_cast<size_t>(IeeestInternalVariables<order>::X1)] = true;
        }
        if constexpr (order >= 2)
        {
          tag_[static_cast<size_t>(IeeestInternalVariables<order>::X2)] = true;
        }
        if constexpr (order >= 3)
        {
          tag_[static_cast<size_t>(IeeestInternalVariables<order>::X3)] = true;
        }
        if constexpr (order >= 4)
        {
          tag_[static_cast<size_t>(IeeestInternalVariables<order>::X4)] = true;
        }

        tag_[static_cast<size_t>(IeeestInternalVariables<order>::X5)] = true;
        tag_[static_cast<size_t>(IeeestInternalVariables<order>::X6)] = true;
        tag_[static_cast<size_t>(IeeestInternalVariables<order>::X7)] = true;

        return 0;
      }

      /**
       * @brief Compute the absolute tolerance for each variable in the model
       *
       * @param rel_tol The relative tolerance which can be used to pick the
       *        absolute tolerance.
       * @tparam scalar_type Scalar data type
       * @tparam index_type Index data type
       * @return int 0 if successful, non-zero otherwise.
       *
       * This represents a "noise" level close to zero for which pure relative
       * error cannot be used.
       */
      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::setAbsoluteTolerance(RealT rel_tol)
      {
        std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      __attribute__((always_inline)) inline int Ieeest<scalar_type, index_type, order>::evaluateInternalResidual(
          const ScalarT*                  y,
          const ScalarT*                  yp,
          [[maybe_unused]] const ScalarT* wb,
          const ScalarT*                  ws,
          ScalarT*                        f)
      {
        const auto X5  = static_cast<size_t>(IeeestInternalVariables<order>::X5);
        const auto X6  = static_cast<size_t>(IeeestInternalVariables<order>::X6);
        const auto X7  = static_cast<size_t>(IeeestInternalVariables<order>::X7);
        const auto V4  = static_cast<size_t>(IeeestInternalVariables<order>::V4);
        const auto V5  = static_cast<size_t>(IeeestInternalVariables<order>::V5);
        const auto V6  = static_cast<size_t>(IeeestInternalVariables<order>::V6);
        const auto V7  = static_cast<size_t>(IeeestInternalVariables<order>::V7);
        const auto VSS = static_cast<size_t>(IeeestInternalVariables<order>::VSS);
        const auto U   = static_cast<size_t>(IeeestExternalVariables::U);

        const ScalarT x5  = y[X5];
        const ScalarT x6  = y[X6];
        const ScalarT x7  = y[X7];
        const ScalarT v4  = y[V4];
        const ScalarT v5  = y[V5];
        const ScalarT v6  = y[V6];
        const ScalarT v7  = y[V7];
        const ScalarT vss = y[VSS];

        const ScalarT x5_dot = yp[X5];
        const ScalarT x6_dot = yp[X6];
        const ScalarT x7_dot = yp[X7];

        const ScalarT u = ws[U];

        // Notch filter -- order-specific realization
        if constexpr (order == 0)
        {
          f[V4] = -v4 + u;
        }
        else if constexpr (order == 1)
        {
          const auto X1 = static_cast<size_t>(IeeestInternalVariables<order>::X1);

          const ScalarT x1     = y[X1];
          const ScalarT x1_dot = yp[X1];

          const ScalarT x1_rhs = (u - x1) / a1_;

          f[X1] = -x1_dot + x1_rhs;
          f[V4] = -v4 + x1 + A5_ * x1_rhs;
        }
        else if constexpr (order == 2)
        {
          const auto X1 = static_cast<size_t>(IeeestInternalVariables<order>::X1);
          const auto X2 = static_cast<size_t>(IeeestInternalVariables<order>::X2);

          const ScalarT x1     = y[X1];
          const ScalarT x2     = y[X2];
          const ScalarT x1_dot = yp[X1];
          const ScalarT x2_dot = yp[X2];

          const ScalarT x2_rhs = (u - x1 - a1_ * x2) / a2_;

          f[X1] = -x1_dot + x2;
          f[X2] = -x2_dot + x2_rhs;
          f[V4] = -v4 + x1 + A5_ * x2 + A6_ * x2_rhs;
        }
        else if constexpr (order == 3)
        {
          const auto X1 = static_cast<size_t>(IeeestInternalVariables<order>::X1);
          const auto X2 = static_cast<size_t>(IeeestInternalVariables<order>::X2);
          const auto X3 = static_cast<size_t>(IeeestInternalVariables<order>::X3);

          const ScalarT x1     = y[X1];
          const ScalarT x2     = y[X2];
          const ScalarT x3     = y[X3];
          const ScalarT x1_dot = yp[X1];
          const ScalarT x2_dot = yp[X2];
          const ScalarT x3_dot = yp[X3];

          const ScalarT x3_rhs = (u - x1 - a1_ * x2 - a2_ * x3) / a3_;

          f[X1] = -x1_dot + x2;
          f[X2] = -x2_dot + x3;
          f[X3] = -x3_dot + x3_rhs;
          f[V4] = -v4 + x1 + A5_ * x2 + A6_ * x3;
        }
        else
        {
          const auto X1 = static_cast<size_t>(IeeestInternalVariables<order>::X1);
          const auto X2 = static_cast<size_t>(IeeestInternalVariables<order>::X2);
          const auto X3 = static_cast<size_t>(IeeestInternalVariables<order>::X3);
          const auto X4 = static_cast<size_t>(IeeestInternalVariables<order>::X4);

          const ScalarT x1     = y[X1];
          const ScalarT x2     = y[X2];
          const ScalarT x3     = y[X3];
          const ScalarT x4     = y[X4];
          const ScalarT x1_dot = yp[X1];
          const ScalarT x2_dot = yp[X2];
          const ScalarT x3_dot = yp[X3];
          const ScalarT x4_dot = yp[X4];

          const ScalarT x4_rhs = (u - x1 - a1_ * x2 - a2_ * x3 - a3_ * x4) / a4_;

          f[X1] = -x1_dot + x2;
          f[X2] = -x2_dot + x3;
          f[X3] = -x3_dot + x4;
          f[X4] = -x4_dot + x4_rhs;
          f[V4] = -v4 + x1 + A5_ * x2 + A6_ * x3;
        }

        // Lead-lags and washout -- shared across all orders
        const ScalarT x5_rhs = (v4 - x5) / T2_;
        const ScalarT x6_rhs = (v5 - x6) / T4_;
        const ScalarT x7_rhs = (v6 - x7) / T6_;

        f[X5]  = -x5_dot + x5_rhs;
        f[X6]  = -x6_dot + x6_rhs;
        f[X7]  = -x7_dot + x7_rhs;
        f[V5]  = -v5 + x5 + T1_ * x5_rhs;
        f[V6]  = -v6 + x6 + T3_ * x6_rhs;
        f[V7]  = -v7 + Ks_ * T5_ * x7_rhs;
        f[VSS] = -vss + Math::clamp(v7, Lsmin_, Lsmax_);

        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      int Ieeest<scalar_type, index_type, order>::evaluateResidual()
      {
        const auto U = static_cast<size_t>(IeeestExternalVariables::U);

        std::fill(ws_.begin(), ws_.end(), ZERO<RealT>);
        std::fill(ws_indices_.begin(), ws_indices_.end(), INVALID_INDEX<IdxT>);

        if (signals_.template isAttached<IeeestExternalVariables::U>())
        {
          ws_[U]         = signals_.template readExternalVariable<IeeestExternalVariables::U>();
          ws_indices_[U] = signals_.template readExternalVariableIndex<IeeestExternalVariables::U>();
        }

        evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), f_.data());

        return 0;
      }

      template <typename scalar_type, typename index_type, size_t order>
      const Model::VariableMonitorBase* Ieeest<scalar_type, index_type, order>::getMonitor() const
      {
        return monitor_.get();
      }

      template <typename scalar_type, typename index_type, size_t order>
      void Ieeest<scalar_type, index_type, order>::initializeMonitor()
      {
        using Variable = typename ModelDataT::MonitorableVariables;

        constexpr auto VSS = static_cast<size_t>(IeeestInternalVariables<order>::VSS);

        monitor_->set(Variable::vss, [this]
                      { return y_[VSS]; });
      }

    } // namespace Stabilizer
  } // namespace PhasorDynamics
} // namespace GridKit
