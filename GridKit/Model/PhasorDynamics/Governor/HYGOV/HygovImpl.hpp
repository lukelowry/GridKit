/**
 * @file HygovImpl.hpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Definition of the HYGOV governor model.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <GridKit/Model/PhasorDynamics/Governor/HYGOV/Hygov.hpp>
#include <GridKit/Model/PhasorDynamics/Governor/HYGOV/HygovData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Governor
    {
      using Log = ::GridKit::Utilities::Logger;

      template <typename scalar_type, typename index_type>
      Hygov<scalar_type, index_type>::Hygov()
      {
        size_ = static_cast<IdxT>(HygovInternalVariables::MAXIMUM);
        setDerivedParams();
      }

      template <typename scalar_type, typename index_type>
      Hygov<scalar_type, index_type>::Hygov(const ModelDataT& data)
        : monitor_(std::make_unique<MonitorT>(data))
      {
        initModelParams(data);
        initializeMonitor();

        size_ = static_cast<IdxT>(HygovInternalVariables::MAXIMUM);
        setDerivedParams();
      }

      template <typename scalar_type, typename index_type>
      Hygov<scalar_type, index_type>::~Hygov()
      {
      }

      template <typename scalar_type, typename index_type>
      void Hygov<scalar_type, index_type>::setDerivedParams()
      {
        va_component_base_ = Trate_ * static_cast<RealT>(1.0e6);
      }

      template <typename scalar_type, typename index_type>
      scalar_type Hygov<scalar_type, index_type>::toComponentBase(scalar_type value) const
      {
        return value * va_system_base_ / va_component_base_;
      }

      template <typename scalar_type, typename index_type>
      scalar_type Hygov<scalar_type, index_type>::toSystemBase(scalar_type value) const
      {
        return value / toComponentBase(static_cast<ScalarT>(ONE<RealT>));
      }

      template <typename scalar_type, typename index_type>
      void Hygov<scalar_type, index_type>::initModelParams(const ModelDataT& data)
      {
        using Params = typename ModelDataT::Parameters;

        Trate_ = ZERO<RealT>;
        Rperm_ = static_cast<RealT>(0.04);
        Rtemp_ = static_cast<RealT>(0.3);
        Tr_    = static_cast<RealT>(5.0);
        Tf_    = static_cast<RealT>(0.05);
        Tg_    = static_cast<RealT>(0.5);
        Velm_  = static_cast<RealT>(0.2);
        Gmax_  = ONE<RealT>;
        Gmin_  = ZERO<RealT>;
        Tw_    = ONE<RealT>;
        At_    = static_cast<RealT>(1.2);
        Dturb_ = static_cast<RealT>(0.5);
        Qnl_   = static_cast<RealT>(0.05);
        Tn_    = ZERO<RealT>;
        Tnp_   = ZERO<RealT>;
        db1_   = ZERO<RealT>;
        db2_   = ZERO<RealT>;
        Hdam_  = ONE<RealT>;
        Gv_.fill(ZERO<RealT>);
        Pgv_.fill(ZERO<RealT>);

        if (data.parameters.contains(Params::Trate))
        {
          Trate_ = std::get<RealT>(data.parameters.at(Params::Trate));
        }
        if (data.parameters.contains(Params::Rperm))
        {
          Rperm_ = std::get<RealT>(data.parameters.at(Params::Rperm));
        }
        if (data.parameters.contains(Params::Rtemp))
        {
          Rtemp_ = std::get<RealT>(data.parameters.at(Params::Rtemp));
        }
        if (data.parameters.contains(Params::Tr))
        {
          Tr_ = std::get<RealT>(data.parameters.at(Params::Tr));
        }
        if (data.parameters.contains(Params::Tf))
        {
          Tf_ = std::get<RealT>(data.parameters.at(Params::Tf));
        }
        if (data.parameters.contains(Params::Tg))
        {
          Tg_ = std::get<RealT>(data.parameters.at(Params::Tg));
        }
        if (data.parameters.contains(Params::Velm))
        {
          Velm_ = std::get<RealT>(data.parameters.at(Params::Velm));
        }
        if (data.parameters.contains(Params::Gmax))
        {
          Gmax_ = std::get<RealT>(data.parameters.at(Params::Gmax));
        }
        if (data.parameters.contains(Params::Gmin))
        {
          Gmin_ = std::get<RealT>(data.parameters.at(Params::Gmin));
        }
        if (data.parameters.contains(Params::Tw))
        {
          Tw_ = std::get<RealT>(data.parameters.at(Params::Tw));
        }
        if (data.parameters.contains(Params::At))
        {
          At_ = std::get<RealT>(data.parameters.at(Params::At));
        }
        if (data.parameters.contains(Params::Dturb))
        {
          Dturb_ = std::get<RealT>(data.parameters.at(Params::Dturb));
        }
        if (data.parameters.contains(Params::Qnl))
        {
          Qnl_ = std::get<RealT>(data.parameters.at(Params::Qnl));
        }
        if (data.parameters.contains(Params::Tn))
        {
          Tn_ = std::get<RealT>(data.parameters.at(Params::Tn));
        }
        if (data.parameters.contains(Params::Tnp))
        {
          Tnp_ = std::get<RealT>(data.parameters.at(Params::Tnp));
        }
        if (data.parameters.contains(Params::db1))
        {
          db1_ = std::get<RealT>(data.parameters.at(Params::db1));
        }
        if (data.parameters.contains(Params::db2))
        {
          db2_ = std::get<RealT>(data.parameters.at(Params::db2));
        }
        if (data.parameters.contains(Params::Hdam))
        {
          Hdam_ = std::get<RealT>(data.parameters.at(Params::Hdam));
        }
        if (data.parameters.contains(Params::Gv0))
        {
          Gv_[0] = std::get<RealT>(data.parameters.at(Params::Gv0));
        }
        if (data.parameters.contains(Params::Gv1))
        {
          Gv_[1] = std::get<RealT>(data.parameters.at(Params::Gv1));
        }
        if (data.parameters.contains(Params::Gv2))
        {
          Gv_[2] = std::get<RealT>(data.parameters.at(Params::Gv2));
        }
        if (data.parameters.contains(Params::Gv3))
        {
          Gv_[3] = std::get<RealT>(data.parameters.at(Params::Gv3));
        }
        if (data.parameters.contains(Params::Gv4))
        {
          Gv_[4] = std::get<RealT>(data.parameters.at(Params::Gv4));
        }
        if (data.parameters.contains(Params::Gv5))
        {
          Gv_[5] = std::get<RealT>(data.parameters.at(Params::Gv5));
        }
        if (data.parameters.contains(Params::Pgv0))
        {
          Pgv_[0] = std::get<RealT>(data.parameters.at(Params::Pgv0));
        }
        if (data.parameters.contains(Params::Pgv1))
        {
          Pgv_[1] = std::get<RealT>(data.parameters.at(Params::Pgv1));
        }
        if (data.parameters.contains(Params::Pgv2))
        {
          Pgv_[2] = std::get<RealT>(data.parameters.at(Params::Pgv2));
        }
        if (data.parameters.contains(Params::Pgv3))
        {
          Pgv_[3] = std::get<RealT>(data.parameters.at(Params::Pgv3));
        }
        if (data.parameters.contains(Params::Pgv4))
        {
          Pgv_[4] = std::get<RealT>(data.parameters.at(Params::Pgv4));
        }
        if (data.parameters.contains(Params::Pgv5))
        {
          Pgv_[5] = std::get<RealT>(data.parameters.at(Params::Pgv5));
        }

        const bool source_default_curve =
            std::all_of(Gv_.begin(), Gv_.end(), [](RealT value)
                        { return value == ZERO<RealT>; })
            && std::all_of(Pgv_.begin(), Pgv_.end(), [](RealT value)
                           { return value == ZERO<RealT>; });
        if (source_default_curve)
        {
          Gv_  = {ZERO<RealT>,
                  static_cast<RealT>(0.2),
                  static_cast<RealT>(0.4),
                  static_cast<RealT>(0.6),
                  static_cast<RealT>(0.8),
                  ONE<RealT>};
          Pgv_ = Gv_;
        }

        Tr_  = std::max(Tr_, TIME_CONSTANT_MINIMUM);
        Tf_  = std::max(Tf_, TIME_CONSTANT_MINIMUM);
        Tg_  = std::max(Tg_, TIME_CONSTANT_MINIMUM);
        Tw_  = std::max(Tw_, TIME_CONSTANT_MINIMUM);
        Tnp_ = std::max(Tnp_, TIME_CONSTANT_MINIMUM);

        leadlag_gain_ = Tn_ / Tnp_;
      }

      template <typename scalar_type, typename index_type>
      const Model::VariableMonitorBase* Hygov<scalar_type, index_type>::getMonitor() const
      {
        return monitor_.get();
      }

      template <typename scalar_type, typename index_type>
      void Hygov<scalar_type, index_type>::initializeMonitor()
      {
        using Variable = typename ModelDataT::MonitorableVariables;
        auto index     = [](HygovInternalVariables variable)
        {
          return static_cast<size_t>(variable);
        };

        monitor_->set(Variable::pmech, [this, index]
                      { return y_[index(HygovInternalVariables::PMECH)]; });
        monitor_->set(Variable::filter, [this, index]
                      { return y_[index(HygovInternalVariables::XF)]; });
        monitor_->set(Variable::desiredgate, [this, index]
                      { return y_[index(HygovInternalVariables::C)]; });
        monitor_->set(Variable::gate, [this, index]
                      { return y_[index(HygovInternalVariables::G)]; });
        monitor_->set(Variable::flow, [this, index]
                      { return y_[index(HygovInternalVariables::Q)]; });
        monitor_->set(Variable::head, [this, index]
                      { return y_[index(HygovInternalVariables::H)]; });
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::allocate()
      {
        size_     = static_cast<IdxT>(HygovInternalVariables::MAXIMUM);
        auto size = static_cast<size_t>(size_);

        f_.resize(size);
        y_.resize(size);
        yp_.resize(size);
        tag_.resize(size);
        abs_tol_.resize(size);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        auto signal_size = static_cast<size_t>(HygovExternalVariables::MAXIMUM);
        ws_.resize(signal_size);
        ws_indices_.resize(signal_size);
        std::fill(ws_.begin(), ws_.end(), ScalarT{0});
        std::fill(ws_indices_.begin(), ws_indices_.end(), INVALID_INDEX<IdxT>);

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        if (signals_.template isAssigned<HygovInternalVariables::PMECH>())
        {
          signals_.template getSignalNode<HygovInternalVariables::PMECH>()->set(
              &y_[static_cast<size_t>(HygovInternalVariables::PMECH)],
              &(this->getVariableIndex(static_cast<IdxT>(HygovInternalVariables::PMECH))));
        }

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::verify() const
      {
        int ret = 0;

        auto check = [&](bool condition, const char* message)
        {
          if (!condition)
          {
            Log::error() << "Hygov: " << message << '\n';
            ret += 1;
          }
        };

        check(Trate_ > ZERO<RealT>, "Trate must be positive");
        check(Rtemp_ != ZERO<RealT>, "Rtemp must be nonzero");
        check(Velm_ >= ZERO<RealT>, "Velm must be non-negative");
        check(Gmin_ <= Gmax_, "Gmin must be less than or equal to Gmax");
        check(db1_ >= ZERO<RealT>, "db1 must be non-negative");
        check(Hdam_ > ZERO<RealT>, "Hdam must be positive");

        for (size_t i = 1; i < Gv_.size(); ++i)
        {
          check(Gv_[i - 1] < Gv_[i], "Gv points must be strictly increasing");
          check(Pgv_[i - 1] <= Pgv_[i], "Pgv points must be non-decreasing");
        }

        if (signals_.template isAttached<HygovExternalVariables::OMEGA>()
            && !signals_.template isLinked<HygovExternalVariables::OMEGA>())
        {
          Log::error() << "Hygov: omega signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<HygovExternalVariables::PREF>()
            && !signals_.template isLinked<HygovExternalVariables::PREF>())
        {
          Log::error() << "Hygov: pref signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<HygovExternalVariables::PAUX>()
            && !signals_.template isLinked<HygovExternalVariables::PAUX>())
        {
          Log::error() << "Hygov: paux signal attached with no linked source\n";
          ret += 1;
        }

        return ret;
      }

      template <typename scalar_type, typename index_type>
      scalar_type Hygov<scalar_type, index_type>::gatePower(scalar_type gate) const
      {
        return ScalarT{Pgv_[0]}
               + Math::linseg(gate, Gv_[0], Gv_[1], Pgv_[1] - Pgv_[0])
               + Math::linseg(gate, Gv_[1], Gv_[2], Pgv_[2] - Pgv_[1])
               + Math::linseg(gate, Gv_[2], Gv_[3], Pgv_[3] - Pgv_[2])
               + Math::linseg(gate, Gv_[3], Gv_[4], Pgv_[4] - Pgv_[3])
               + Math::linseg(gate, Gv_[4], Gv_[5], Pgv_[5] - Pgv_[4]);
      }

      template <typename scalar_type, typename index_type>
      typename Hygov<scalar_type, index_type>::RealT
      Hygov<scalar_type, index_type>::invertGatePower(
          typename Hygov<scalar_type, index_type>::RealT pgv) const
      {
        static constexpr RealT tol = static_cast<RealT>(1.0e-10);

        if (std::abs(pgv - Pgv_[0]) <= tol)
        {
          return Gv_[0];
        }

        for (size_t i = 0; i < 5; ++i)
        {
          if (Pgv_[i + 1] <= Pgv_[i])
          {
            continue;
          }

          if (Pgv_[i] - tol <= pgv && pgv <= Pgv_[i + 1] + tol)
          {
            const RealT fraction = (pgv - Pgv_[i]) / (Pgv_[i + 1] - Pgv_[i]);
            return Gv_[i] + fraction * (Gv_[i + 1] - Gv_[i]);
          }
        }

        return std::numeric_limits<RealT>::quiet_NaN();
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::initialize()
      {
        if (verify() > 0)
        {
          Log::error() << "Hygov: cannot initialize with invalid configuration\n";
          return 1;
        }

        const auto XN      = static_cast<size_t>(HygovInternalVariables::XN);
        const auto XF      = static_cast<size_t>(HygovInternalVariables::XF);
        const auto C       = static_cast<size_t>(HygovInternalVariables::C);
        const auto G       = static_cast<size_t>(HygovInternalVariables::G);
        const auto Q       = static_cast<size_t>(HygovInternalVariables::Q);
        const auto OMEGADB = static_cast<size_t>(HygovInternalVariables::OMEGADB);
        const auto EF      = static_cast<size_t>(HygovInternalVariables::EF);
        const auto FC      = static_cast<size_t>(HygovInternalVariables::FC);
        const auto RC      = static_cast<size_t>(HygovInternalVariables::RC);
        const auto PGV     = static_cast<size_t>(HygovInternalVariables::PGV);
        const auto H       = static_cast<size_t>(HygovInternalVariables::H);
        const auto PMECH   = static_cast<size_t>(HygovInternalVariables::PMECH);

        ScalarT omega0{ZERO<RealT>};
        if (signals_.template isAttached<HygovExternalVariables::OMEGA>())
        {
          omega0 = signals_.template readExternalVariable<HygovExternalVariables::OMEGA>();
        }

        paux_set_ = ScalarT{ZERO<RealT>};
        if (signals_.template isAttached<HygovExternalVariables::PAUX>())
        {
          paux_set_ = signals_.template readExternalVariable<HygovExternalVariables::PAUX>();
        }

        const ScalarT pmech0 = toComponentBase(y_[PMECH]);
        y_[H]                = Hdam_;
        y_[Q]                = Qnl_ + pmech0 / (At_ * y_[H]);
        y_[PGV]              = y_[Q] / std::sqrt(y_[H]);

        const RealT gate0 = invertGatePower(static_cast<RealT>(y_[PGV]));
        if (std::isnan(gate0))
        {
          Log::error() << "Hygov: initial Pgv is outside the invertible gate curve\n";
          return 1;
        }

        y_[G] = gate0;
        y_[C] = y_[G];

        if (y_[C] < Gmin_ || y_[C] > Gmax_)
        {
          Log::error() << "Hygov: initialized gate is outside Gmin/Gmax\n";
          return 1;
        }

        y_[OMEGADB] = Math::deadband1(omega0, -db1_, db1_);
        y_[XN]      = y_[OMEGADB];
        y_[XF]      = ZERO<RealT>;
        y_[EF]      = ZERO<RealT>;
        y_[FC]      = ZERO<RealT>;
        y_[RC]      = ZERO<RealT>;
        y_[PMECH]   = toSystemBase(pmech0);

        const ScalarT yomega = y_[XN] + leadlag_gain_ * (y_[OMEGADB] - y_[XN]);
        pref_set_            = y_[EF] - paux_set_ + yomega + Rperm_ * y_[C];
        if (signals_.template isAttached<HygovExternalVariables::PREF>())
        {
          const ScalarT pref0 =
              signals_.template readExternalVariable<HygovExternalVariables::PREF>();
          const RealT pref_err = static_cast<RealT>(pref0 - pref_set_);
          if (std::abs(pref_err) > static_cast<RealT>(1.0e-10))
          {
            Log::error() << "Hygov: pref initial condition is not steady state\n";
            return 1;
          }
          pref_set_ = pref0;
        }

        std::fill(yp_.begin(), yp_.end(), ZERO<RealT>);
        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::tagDifferentiable()
      {
        std::fill(tag_.begin(), tag_.end(), false);
        tag_[static_cast<size_t>(HygovInternalVariables::XN)] = true;
        tag_[static_cast<size_t>(HygovInternalVariables::XF)] = true;
        tag_[static_cast<size_t>(HygovInternalVariables::C)]  = true;
        tag_[static_cast<size_t>(HygovInternalVariables::G)]  = true;
        tag_[static_cast<size_t>(HygovInternalVariables::Q)]  = true;
        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
      {
        std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
        return 0;
      }

      template <typename scalar_type, typename index_type>
      __attribute__((always_inline)) inline int
      Hygov<scalar_type, index_type>::evaluateInternalResidual(
          const ScalarT*                  y,
          const ScalarT*                  yp,
          [[maybe_unused]] const ScalarT* wb,
          const ScalarT*                  ws,
          ScalarT*                        f)
      {
        const auto XN      = static_cast<size_t>(HygovInternalVariables::XN);
        const auto XF      = static_cast<size_t>(HygovInternalVariables::XF);
        const auto C       = static_cast<size_t>(HygovInternalVariables::C);
        const auto G       = static_cast<size_t>(HygovInternalVariables::G);
        const auto Q       = static_cast<size_t>(HygovInternalVariables::Q);
        const auto OMEGADB = static_cast<size_t>(HygovInternalVariables::OMEGADB);
        const auto EF      = static_cast<size_t>(HygovInternalVariables::EF);
        const auto FC      = static_cast<size_t>(HygovInternalVariables::FC);
        const auto RC      = static_cast<size_t>(HygovInternalVariables::RC);
        const auto PGV     = static_cast<size_t>(HygovInternalVariables::PGV);
        const auto H       = static_cast<size_t>(HygovInternalVariables::H);
        const auto PMECH   = static_cast<size_t>(HygovInternalVariables::PMECH);

        const auto OMEGA = static_cast<size_t>(HygovExternalVariables::OMEGA);
        const auto PREF  = static_cast<size_t>(HygovExternalVariables::PREF);
        const auto PAUX  = static_cast<size_t>(HygovExternalVariables::PAUX);

        const ScalarT xn      = y[XN];
        const ScalarT xf      = y[XF];
        const ScalarT c       = y[C];
        const ScalarT g       = y[G];
        const ScalarT q       = y[Q];
        const ScalarT omegadb = y[OMEGADB];
        const ScalarT ef      = y[EF];
        const ScalarT fc      = y[FC];
        const ScalarT rc      = y[RC];
        const ScalarT pgv     = y[PGV];
        const ScalarT head    = y[H];
        const ScalarT pmech   = y[PMECH];

        const ScalarT omega = ws[OMEGA];
        const ScalarT pref  = ws[PREF];
        const ScalarT paux  = ws[PAUX];

        const ScalarT yomega = xn + leadlag_gain_ * (omegadb - xn);

        f[XN]      = -yp[XN] + (omegadb - xn) / Tnp_;
        f[XF]      = -yp[XF] + (ef - xf) / Tf_;
        f[C]       = -yp[C] + Math::antiwindup(c, rc, Gmin_, Gmax_);
        f[G]       = -yp[G] + (c - g) / Tg_;
        f[Q]       = -yp[Q] + (Hdam_ - head) / Tw_;
        f[OMEGADB] = -omegadb + Math::deadband1(omega, -db1_, db1_);
        f[EF]      = -ef + pref + paux - yomega - Rperm_ * c;
        f[FC]      = -fc + (xf / Tr_ + (ef - xf) / Tf_) / Rtemp_;
        f[RC]      = -rc + Math::clamp(fc, -Velm_, Velm_);
        f[PGV]     = -pgv + gatePower(g);
        f[H]       = -q * q + head * pgv * pgv;
        f[PMECH]   = -toComponentBase(pmech) + At_ * head * (q - Qnl_) - Dturb_ * omega * g;

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Hygov<scalar_type, index_type>::evaluateResidual()
      {
        const auto OMEGA = static_cast<size_t>(HygovExternalVariables::OMEGA);
        const auto PREF  = static_cast<size_t>(HygovExternalVariables::PREF);
        const auto PAUX  = static_cast<size_t>(HygovExternalVariables::PAUX);

        ws_[OMEGA] = ZERO<RealT>;
        ws_[PREF]  = pref_set_;
        ws_[PAUX]  = paux_set_;
        std::fill(ws_indices_.begin(), ws_indices_.end(), INVALID_INDEX<IdxT>);

        if (signals_.template isAttached<HygovExternalVariables::OMEGA>())
        {
          ws_[OMEGA] = signals_.template readExternalVariable<HygovExternalVariables::OMEGA>();
          ws_indices_[OMEGA] =
              signals_.template readExternalVariableIndex<HygovExternalVariables::OMEGA>();
        }

        if (signals_.template isAttached<HygovExternalVariables::PREF>())
        {
          ws_[PREF] = signals_.template readExternalVariable<HygovExternalVariables::PREF>();
          ws_indices_[PREF] =
              signals_.template readExternalVariableIndex<HygovExternalVariables::PREF>();
        }

        if (signals_.template isAttached<HygovExternalVariables::PAUX>())
        {
          ws_[PAUX] = signals_.template readExternalVariable<HygovExternalVariables::PAUX>();
          ws_indices_[PAUX] =
              signals_.template readExternalVariableIndex<HygovExternalVariables::PAUX>();
        }

        evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), f_.data());
        return 0;
      }
    } // namespace Governor
  } // namespace PhasorDynamics
} // namespace GridKit
