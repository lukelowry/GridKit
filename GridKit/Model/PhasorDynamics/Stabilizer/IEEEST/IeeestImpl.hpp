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

      template <typename scalar_type, typename index_type>
      Ieeest<scalar_type, index_type>::Ieeest()
      {
        size_ = static_cast<IdxT>(IeeestInternalVariables::MAXIMUM);
        setDerivedParameters();
      }

      template <typename scalar_type, typename index_type>
      Ieeest<scalar_type, index_type>::Ieeest(const ModelDataT& data)
        : monitor_(std::make_unique<MonitorT>(data))
      {
        initializeParameters(data);
        initializeMonitor();
        size_ = static_cast<IdxT>(IeeestInternalVariables::MAXIMUM);
        setDerivedParameters();
      }

      template <typename scalar_type, typename index_type>
      Ieeest<scalar_type, index_type>::~Ieeest()
      {
      }

      template <typename scalar_type, typename index_type>
      void Ieeest<scalar_type, index_type>::setDerivedParameters()
      {
        T2_ = std::max(T2_, TIME_CONSTANT_MINIMUM);
        T4_ = std::max(T4_, TIME_CONSTANT_MINIMUM);
        T6_ = std::max(T6_, TIME_CONSTANT_MINIMUM);

        a1_ = A1_ + A3_;
        a2_ = A2_ + A4_ + A1_ * A3_;
        a3_ = A1_ * A4_ + A2_ * A3_;
        a4_ = A2_ * A4_;

        order_ = 0;
        if (a1_ != ZERO<RealT>)
        {
          order_ = 1;
        }
        if (a2_ != ZERO<RealT>)
        {
          order_ = 2;
        }
        if (a3_ != ZERO<RealT>)
        {
          order_ = 3;
        }
        if (a4_ != ZERO<RealT>)
        {
          order_ = 4;
        }

        s0_ = (order_ == 0) ? ONE<RealT> : ZERO<RealT>;
        s1_ = (order_ == 1) ? ONE<RealT> : ZERO<RealT>;
        s2_ = (order_ == 2) ? ONE<RealT> : ZERO<RealT>;
        s3_ = (order_ == 3) ? ONE<RealT> : ZERO<RealT>;
        s4_ = (order_ == 4) ? ONE<RealT> : ZERO<RealT>;

        a1_inv_ = (order_ == 1) ? ONE<RealT> / a1_ : ZERO<RealT>;
        a2_inv_ = (order_ == 2) ? ONE<RealT> / a2_ : ZERO<RealT>;
        a3_inv_ = (order_ == 3) ? ONE<RealT> / a3_ : ZERO<RealT>;
        a4_inv_ = (order_ == 4) ? ONE<RealT> / a4_ : ZERO<RealT>;
      }

      template <typename scalar_type, typename index_type>
      void Ieeest<scalar_type, index_type>::initializeParameters(const ModelDataT& data)
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

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::allocate()
      {
        size_     = static_cast<IdxT>(IeeestInternalVariables::MAXIMUM);
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

        if (signals_.template isAssigned<IeeestInternalVariables::VSS>())
        {
          signals_.template getSignalNode<IeeestInternalVariables::VSS>()->set(
              &y_[static_cast<size_t>(IeeestInternalVariables::VSS)],
              &(this->getVariableIndex(static_cast<IdxT>(IeeestInternalVariables::VSS))));
        }

        tagDifferentiable();

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::verify() const
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

        if (order_ == 1 && A6_ != ZERO<RealT>)
        {
          Log::error() << "Ieeest: unsupported first-order notch filter with second-order numerator\n";
          ret += 1;
        }

        return ret;
      }

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::initialize()
      {
        if (verify() > 0)
        {
          Log::error() << "Ieeest: cannot initialize with invalid configuration\n";
          return 1;
        }

        const auto X1  = static_cast<size_t>(IeeestInternalVariables::X1);
        const auto X2  = static_cast<size_t>(IeeestInternalVariables::X2);
        const auto X3  = static_cast<size_t>(IeeestInternalVariables::X3);
        const auto X4  = static_cast<size_t>(IeeestInternalVariables::X4);
        const auto X5  = static_cast<size_t>(IeeestInternalVariables::X5);
        const auto X6  = static_cast<size_t>(IeeestInternalVariables::X6);
        const auto X7  = static_cast<size_t>(IeeestInternalVariables::X7);
        const auto V4  = static_cast<size_t>(IeeestInternalVariables::V4);
        const auto V5  = static_cast<size_t>(IeeestInternalVariables::V5);
        const auto V6  = static_cast<size_t>(IeeestInternalVariables::V6);
        const auto V7  = static_cast<size_t>(IeeestInternalVariables::V7);
        const auto VSS = static_cast<size_t>(IeeestInternalVariables::VSS);
        const auto U   = static_cast<size_t>(IeeestExternalVariables::U);

        std::fill(y_.begin(), y_.end(), ZERO<RealT>);
        std::fill(yp_.begin(), yp_.end(), ZERO<RealT>);

        const ScalarT u = signals_.template readExternalVariable<IeeestExternalVariables::U>();
        ws_[U]          = u;
        ws_indices_[U]  = signals_.template readExternalVariableIndex<IeeestExternalVariables::U>();

        y_[X1]  = u;
        y_[X2]  = ZERO<RealT>;
        y_[X3]  = ZERO<RealT>;
        y_[X4]  = ZERO<RealT>;
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

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::tagDifferentiable()
      {
        auto index = [](IeeestInternalVariables variable)
        {
          return static_cast<size_t>(variable);
        };

        std::fill(tag_.begin(), tag_.end(), false);
        tag_[index(IeeestInternalVariables::X1)] = true;
        tag_[index(IeeestInternalVariables::X2)] = true;
        tag_[index(IeeestInternalVariables::X3)] = true;
        tag_[index(IeeestInternalVariables::X4)] = true;
        tag_[index(IeeestInternalVariables::X5)] = true;
        tag_[index(IeeestInternalVariables::X6)] = true;
        tag_[index(IeeestInternalVariables::X7)] = true;

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
      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
      {
        std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
        return 0;
      }

      template <typename scalar_type, typename index_type>
      __attribute__((always_inline)) inline int Ieeest<scalar_type, index_type>::evaluateInternalResidual(
          const ScalarT*                  y,
          const ScalarT*                  yp,
          [[maybe_unused]] const ScalarT* wb,
          const ScalarT*                  ws,
          ScalarT*                        f)
      {
        const auto X1  = static_cast<size_t>(IeeestInternalVariables::X1);
        const auto X2  = static_cast<size_t>(IeeestInternalVariables::X2);
        const auto X3  = static_cast<size_t>(IeeestInternalVariables::X3);
        const auto X4  = static_cast<size_t>(IeeestInternalVariables::X4);
        const auto X5  = static_cast<size_t>(IeeestInternalVariables::X5);
        const auto X6  = static_cast<size_t>(IeeestInternalVariables::X6);
        const auto X7  = static_cast<size_t>(IeeestInternalVariables::X7);
        const auto V4  = static_cast<size_t>(IeeestInternalVariables::V4);
        const auto V5  = static_cast<size_t>(IeeestInternalVariables::V5);
        const auto V6  = static_cast<size_t>(IeeestInternalVariables::V6);
        const auto V7  = static_cast<size_t>(IeeestInternalVariables::V7);
        const auto VSS = static_cast<size_t>(IeeestInternalVariables::VSS);
        const auto U   = static_cast<size_t>(IeeestExternalVariables::U);

        const ScalarT x1  = y[X1];
        const ScalarT x2  = y[X2];
        const ScalarT x3  = y[X3];
        const ScalarT x4  = y[X4];
        const ScalarT x5  = y[X5];
        const ScalarT x6  = y[X6];
        const ScalarT x7  = y[X7];
        const ScalarT v4  = y[V4];
        const ScalarT v5  = y[V5];
        const ScalarT v6  = y[V6];
        const ScalarT v7  = y[V7];
        const ScalarT vss = y[VSS];

        const ScalarT x1_dot = yp[X1];
        const ScalarT x2_dot = yp[X2];
        const ScalarT x3_dot = yp[X3];
        const ScalarT x4_dot = yp[X4];
        const ScalarT x5_dot = yp[X5];
        const ScalarT x6_dot = yp[X6];
        const ScalarT x7_dot = yp[X7];

        const ScalarT u = ws[U];

        const RealT s0 = s0_;
        const RealT s1 = s1_;
        const RealT s2 = s2_;
        const RealT s3 = s3_;
        const RealT s4 = s4_;

        const ScalarT x1_rhs = (-x1 + u) * a1_inv_;
        const ScalarT x2_rhs = (-x1 - a1_ * x2 + u) * a2_inv_;
        const ScalarT x3_rhs = (-x1 - a1_ * x2 - a2_ * x3 + u) * a3_inv_;
        const ScalarT x4_rhs = (-x1 - a1_ * x2 - a2_ * x3 - a3_ * x4 + u) * a4_inv_;
        const ScalarT x5_rhs = (v4 - x5) / T2_;
        const ScalarT x6_rhs = (v5 - x6) / T4_;
        const ScalarT x7_rhs = (v6 - x7) / T6_;

        f[X1] = -x1_dot + s1 * x1_rhs + (s2 + s3 + s4) * x2;
        f[X2] = -x2_dot + s2 * x2_rhs + (s3 + s4) * x3;
        f[X3] = -x3_dot + s3 * x3_rhs + s4 * x4;
        f[X4] = -x4_dot + s4 * x4_rhs;
        f[X5] = -x5_dot + x5_rhs;
        f[X6] = -x6_dot + x6_rhs;
        f[X7] = -x7_dot + x7_rhs;

        f[V4] = -v4 + s0 * u
                + s1 * (x1 + A5_ * x1_rhs)
                + s2 * (x1 + A5_ * x2 + A6_ * x2_rhs)
                + (s3 + s4) * (x1 + A5_ * x2 + A6_ * x3);
        f[V5]  = -v5 + x5 + T1_ * x5_rhs;
        f[V6]  = -v6 + x6 + T3_ * x6_rhs;
        f[V7]  = -v7 + Ks_ * T5_ * x7_rhs;
        f[VSS] = -vss + Math::clamp(v7, Lsmin_, Lsmax_);

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Ieeest<scalar_type, index_type>::evaluateResidual()
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

      template <typename scalar_type, typename index_type>
      const Model::VariableMonitorBase* Ieeest<scalar_type, index_type>::getMonitor() const
      {
        return monitor_.get();
      }

      template <typename scalar_type, typename index_type>
      void Ieeest<scalar_type, index_type>::initializeMonitor()
      {
        using Variable = typename ModelDataT::MonitorableVariables;
        auto index     = [](IeeestInternalVariables variable)
        {
          return static_cast<size_t>(variable);
        };

        monitor_->set(Variable::vss, [this, index]
                      { return y_[index(IeeestInternalVariables::VSS)]; });
      }

    } // namespace Stabilizer
  } // namespace PhasorDynamics
} // namespace GridKit
