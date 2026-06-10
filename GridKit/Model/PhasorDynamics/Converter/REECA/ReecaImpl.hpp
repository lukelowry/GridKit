/**
 * @file ReecaImpl.hpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Definition of the REECA electrical-control model.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <variant>

#include <GridKit/Model/PhasorDynamics/BusBase.hpp>
#include <GridKit/Model/PhasorDynamics/Converter/REECA/Reeca.hpp>
#include <GridKit/Model/PhasorDynamics/Converter/REECA/ReecaData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Converter
    {
      using Log = ::GridKit::Utilities::Logger;

      template <typename scalar_type, typename index_type>
      Reeca<scalar_type, index_type>::Reeca(BusT* bus)
        : bus_(bus)
      {
        size_ = static_cast<IdxT>(ReecaInternalVariables::MAXIMUM);
      }

      template <typename scalar_type, typename index_type>
      Reeca<scalar_type, index_type>::Reeca(BusT* bus, const ModelDataT& data)
        : bus_(bus),
          monitor_(std::make_unique<MonitorT>(data))
      {
        initModelParams(data);
        initializeMonitor();
        size_ = static_cast<IdxT>(ReecaInternalVariables::MAXIMUM);
      }

      template <typename scalar_type, typename index_type>
      Reeca<scalar_type, index_type>::~Reeca()
      {
      }

      template <typename scalar_type, typename index_type>
      scalar_type& Reeca<scalar_type, index_type>::Vr()
      {
        return bus_->Vr();
      }

      template <typename scalar_type, typename index_type>
      scalar_type& Reeca<scalar_type, index_type>::Vi()
      {
        return bus_->Vi();
      }

      template <typename scalar_type, typename index_type>
      void Reeca<scalar_type, index_type>::setDerivedParameters()
      {
        va_converter_base_ = mva_base_ * static_cast<RealT>(1.0e6);
        pf_off_            = ONE<RealT> - PfFlag_;
        v_off_             = ONE<RealT> - VFlag_;
        q_off_             = ONE<RealT> - QFlag_;
      }

      template <typename scalar_type, typename index_type>
      scalar_type Reeca<scalar_type, index_type>::toComponentBase(
          scalar_type value) const
      {
        return value * va_system_base_ / va_converter_base_;
      }

      template <typename scalar_type, typename index_type>
      scalar_type Reeca<scalar_type, index_type>::toSystemBase(
          scalar_type value) const
      {
        return value * va_converter_base_ / va_system_base_;
      }

      template <typename scalar_type, typename index_type>
      void Reeca<scalar_type, index_type>::initModelParams(const ModelDataT& data)
      {
        using Params = typename ModelDataT::Parameters;

        parameter_error_count_ = 0;
        Vref0_given_           = false;

        auto load_required_real = [&](auto key, RealT& target, const char* name)
        {
          if (!data.parameters.contains(key))
          {
            Log::error() << "Reeca: missing required parameter '" << name << "'\n";
            ++parameter_error_count_;
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
            Log::error() << "Reeca: parameter '" << name << "' must be numeric\n";
            ++parameter_error_count_;
          }
        };

        auto load_required_switch = [&](auto key, RealT& target, const char* name)
        {
          if (!data.parameters.contains(key))
          {
            Log::error() << "Reeca: missing required parameter '" << name << "'\n";
            ++parameter_error_count_;
            return;
          }

          const auto& value = data.parameters.at(key);
          if (const auto* bool_value = std::get_if<bool>(&value))
          {
            target = ZERO<RealT>;
            if (*bool_value)
            {
              target = ONE<RealT>;
            }
          }
          else if (const auto* index_value = std::get_if<IdxT>(&value);
                   index_value && (*index_value == 0 || *index_value == 1))
          {
            target = static_cast<RealT>(*index_value);
          }
          else if (const auto* real_value = std::get_if<RealT>(&value);
                   real_value && (*real_value == ZERO<RealT> || *real_value == ONE<RealT>) )
          {
            target = *real_value;
          }
          else
          {
            Log::error() << "Reeca: parameter '" << name << "' must be bool or 0/1\n";
            ++parameter_error_count_;
          }
        };

        load_required_real(Params::mva, mva_base_, "mva");
        load_required_switch(Params::PfFlag, PfFlag_, "PfFlag");
        load_required_switch(Params::VFlag, VFlag_, "VFlag");
        load_required_switch(Params::QFlag, QFlag_, "QFlag");
        load_required_switch(Params::PFlag, PFlag_, "PFlag");
        load_required_switch(Params::Pqflag, Pqflag_, "Pqflag");
        load_required_real(Params::Trv, Trv_, "Trv");
        load_required_real(Params::Tp, Tp_, "Tp");
        if (data.parameters.contains(Params::Vref0))
        {
          load_required_real(Params::Vref0, Vref0_, "Vref0");
          Vref0_given_ = true;
        }
        load_required_real(Params::Vdip, Vdip_, "Vdip");
        load_required_real(Params::Vup, Vup_, "Vup");
        load_required_real(Params::dbd1, dbd1_, "dbd1");
        load_required_real(Params::dbd2, dbd2_, "dbd2");
        load_required_real(Params::kqv, kqv_, "kqv");
        load_required_real(Params::Iql1, Iql1_, "Iql1");
        load_required_real(Params::Iqh1, Iqh1_, "Iqh1");
        load_required_real(Params::Iqfrz, Iqfrz_, "Iqfrz");
        load_required_real(Params::Thld, Thld_, "Thld");
        if (data.parameters.contains(Params::Thld2))
        {
          load_required_real(Params::Thld2, Thld2_, "Thld2");
        }
        load_required_real(Params::Qmax, Qmax_, "Qmax");
        load_required_real(Params::Qmin, Qmin_, "Qmin");
        load_required_real(Params::Kqp, Kqp_, "Kqp");
        load_required_real(Params::Kqi, Kqi_, "Kqi");
        load_required_real(Params::Vmax, Vmax_, "Vmax");
        load_required_real(Params::Vmin, Vmin_, "Vmin");
        load_required_real(Params::Vref1, Vref1_, "Vref1");
        load_required_real(Params::Kvp, Kvp_, "Kvp");
        load_required_real(Params::Kvi, Kvi_, "Kvi");
        load_required_real(Params::Tiq, Tiq_, "Tiq");
        load_required_real(Params::Tpord, Tpord_, "Tpord");
        load_required_real(Params::dPmax, dPmax_, "dPmax");
        load_required_real(Params::dPmin, dPmin_, "dPmin");
        load_required_real(Params::Pmax, Pmax_, "Pmax");
        load_required_real(Params::Pmin, Pmin_, "Pmin");
        load_required_real(Params::Imax, Imax_, "Imax");
        load_required_real(Params::Vq1, Vq1_, "Vq1");
        load_required_real(Params::Iq1, Iq1_, "Iq1");
        load_required_real(Params::Vq2, Vq2_, "Vq2");
        load_required_real(Params::Iq2, Iq2_, "Iq2");
        load_required_real(Params::Vq3, Vq3_, "Vq3");
        load_required_real(Params::Iq3, Iq3_, "Iq3");
        load_required_real(Params::Vq4, Vq4_, "Vq4");
        load_required_real(Params::Iq4, Iq4_, "Iq4");
        load_required_real(Params::Vp1, Vp1_, "Vp1");
        load_required_real(Params::Ip1, Ip1_, "Ip1");
        load_required_real(Params::Vp2, Vp2_, "Vp2");
        load_required_real(Params::Ip2, Ip2_, "Ip2");
        load_required_real(Params::Vp3, Vp3_, "Vp3");
        load_required_real(Params::Ip3, Ip3_, "Ip3");
        load_required_real(Params::Vp4, Vp4_, "Vp4");
        load_required_real(Params::Ip4, Ip4_, "Ip4");
        setDerivedParameters();
      }

      template <typename scalar_type, typename index_type>
      const Model::VariableMonitorBase* Reeca<scalar_type, index_type>::getMonitor() const
      {
        return monitor_.get();
      }

      template <typename scalar_type, typename index_type>
      void Reeca<scalar_type, index_type>::initializeMonitor()
      {
        using Variable = typename ModelDataT::MonitorableVariables;
        auto index     = [](ReecaInternalVariables variable)
        {
          return static_cast<size_t>(variable);
        };

        monitor_->set(Variable::iqcmd, [this, index]
                      { return y_[index(ReecaInternalVariables::IQCMD)]; });
        monitor_->set(Variable::ipcmd, [this, index]
                      { return y_[index(ReecaInternalVariables::IPCMD)]; });
        monitor_->set(Variable::vmeas, [this, index]
                      { return y_[index(ReecaInternalVariables::VMEAS)]; });
        monitor_->set(Variable::pmeas, [this, index]
                      { return y_[index(ReecaInternalVariables::PMEAS)]; });
        monitor_->set(Variable::piq, [this, index]
                      { return y_[index(ReecaInternalVariables::XPIQ)]; });
        monitor_->set(Variable::piv, [this, index]
                      { return y_[index(ReecaInternalVariables::XPIV)]; });
        monitor_->set(Variable::qv, [this, index]
                      { return y_[index(ReecaInternalVariables::QV)]; });
        monitor_->set(Variable::pord, [this, index]
                      { return y_[index(ReecaInternalVariables::PORD)]; });
        monitor_->set(Variable::qref, [this, index]
                      { return y_[index(ReecaInternalVariables::QREF)]; });
        monitor_->set(Variable::sdip, [this, index]
                      { return y_[index(ReecaInternalVariables::SDIP)]; });
        monitor_->set(Variable::iqmax, [this, index]
                      { return y_[index(ReecaInternalVariables::IQMAX)]; });
        monitor_->set(Variable::ipmax, [this, index]
                      { return y_[index(ReecaInternalVariables::IPMAX)]; });
        monitor_->set(Variable::iqv, [this, index]
                      { return y_[index(ReecaInternalVariables::IQV)]; });
        monitor_->set(Variable::vqctrl, [this, index]
                      { return y_[index(ReecaInternalVariables::VPIQ)]; });
        monitor_->set(Variable::iqbase, [this, index]
                      { return y_[index(ReecaInternalVariables::IQBASE)]; });
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::allocate()
      {
        size_     = static_cast<IdxT>(ReecaInternalVariables::MAXIMUM);
        auto size = static_cast<size_t>(size_);

        f_.assign(size, ScalarT{0});
        y_.assign(size, ScalarT{0});
        yp_.assign(size, ScalarT{0});
        tag_.assign(size, false);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        wb_.assign(2, ScalarT{0});

        auto signal_size = static_cast<size_t>(ReecaExternalVariables::MAXIMUM);
        ws_.assign(signal_size, ScalarT{0});
        ws_indices_.assign(signal_size, INVALID_INDEX<IdxT>);

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        if (signals_.template isAssigned<ReecaInternalVariables::IQCMD>())
        {
          signals_.template getSignalNode<ReecaInternalVariables::IQCMD>()->set(
              &y_[static_cast<size_t>(ReecaInternalVariables::IQCMD)],
              &(this->getVariableIndex(static_cast<IdxT>(ReecaInternalVariables::IQCMD))));
        }

        if (signals_.template isAssigned<ReecaInternalVariables::IPCMD>())
        {
          signals_.template getSignalNode<ReecaInternalVariables::IPCMD>()->set(
              &y_[static_cast<size_t>(ReecaInternalVariables::IPCMD)],
              &(this->getVariableIndex(static_cast<IdxT>(ReecaInternalVariables::IPCMD))));
        }

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::verify() const
      {
        int ret = static_cast<int>(parameter_error_count_);

        auto check = [&](bool condition, const char* message)
        {
          if (!condition)
          {
            Log::error() << "Reeca: " << message << '\n';
            ret += 1;
          }
        };

        if (bus_ == nullptr)
        {
          Log::error() << "Reeca: bus pointer is null\n";
          ret += 1;
        }

        check(mva_base_ > ZERO<RealT>, "mva must be positive");
        check(va_converter_base_ > ZERO<RealT>, "converter VA base must be positive");
        check(PfFlag_ == ZERO<RealT> || PfFlag_ == ONE<RealT>, "PfFlag must be 0 or 1");
        check(VFlag_ == ZERO<RealT> || VFlag_ == ONE<RealT>, "VFlag must be 0 or 1");
        check(QFlag_ == ZERO<RealT> || QFlag_ == ONE<RealT>, "QFlag must be 0 or 1");
        check(PFlag_ == ZERO<RealT> || PFlag_ == ONE<RealT>, "PFlag must be 0 or 1");
        check(Pqflag_ == ZERO<RealT> || Pqflag_ == ONE<RealT>, "Pqflag must be 0 or 1");
        check(Trv_ >= ZERO<RealT>, "Trv must be non-negative");
        check(Tp_ >= ZERO<RealT>, "Tp must be non-negative");
        check(ZERO<RealT> <= Vdip_ && Vdip_ < Vup_, "Vdip/Vup must satisfy 0 <= Vdip < Vup");
        check(dbd1_ <= ZERO<RealT> && ZERO<RealT> <= dbd2_, "dbd1 <= 0 <= dbd2 is required");
        check(Iql1_ <= Iqh1_, "Iql1 must be less than or equal to Iqh1");
        check(Thld_ == ZERO<RealT>, "Thld must be zero in this REECA implementation");
        check(Thld2_ == ZERO<RealT>, "Thld2 must be zero in this REECA implementation");
        check(Qmin_ <= Qmax_, "Qmin must be less than or equal to Qmax");
        check(Vmin_ <= Vmax_, "Vmin must be less than or equal to Vmax");
        check(Tiq_ > ZERO<RealT>, "Tiq must be positive");
        check(Tpord_ > ZERO<RealT>, "Tpord must be positive");
        check(dPmin_ < ZERO<RealT> && ZERO<RealT> < dPmax_, "dPmin < 0 < dPmax is required");
        check(Pmin_ <= Pmax_, "Pmin must be less than or equal to Pmax");
        check(Imax_ >= ZERO<RealT>, "Imax must be non-negative");
        check(ZERO<RealT> <= Vq1_ && Vq1_ < Vq2_ && Vq2_ < Vq3_ && Vq3_ < Vq4_,
              "Vq breakpoints must satisfy 0 <= Vq1 < Vq2 < Vq3 < Vq4");
        check(Iq1_ >= ZERO<RealT> && Iq2_ >= ZERO<RealT> && Iq3_ >= ZERO<RealT>
                  && Iq4_ >= ZERO<RealT>,
              "Iq VDL limits must be non-negative");
        check(ZERO<RealT> <= Vp1_ && Vp1_ < Vp2_ && Vp2_ < Vp3_ && Vp3_ < Vp4_,
              "Vp breakpoints must satisfy 0 <= Vp1 < Vp2 < Vp3 < Vp4");
        check(Ip1_ >= ZERO<RealT> && Ip2_ >= ZERO<RealT> && Ip3_ >= ZERO<RealT>
                  && Ip4_ >= ZERO<RealT>,
              "Ip VDL limits must be non-negative");

        const bool has_pe   = signals_.template isAttached<ReecaExternalVariables::PE>();
        const bool has_qgen = signals_.template isAttached<ReecaExternalVariables::QGEN>();

        if (has_pe != has_qgen)
        {
          Log::error() << "Reeca: pe and qgen must be connected together\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::PE>()
            && !signals_.template isLinked<ReecaExternalVariables::PE>())
        {
          Log::error() << "Reeca: pe signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::QGEN>()
            && !signals_.template isLinked<ReecaExternalVariables::QGEN>())
        {
          Log::error() << "Reeca: qgen signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::OMEGA>()
            && !signals_.template isLinked<ReecaExternalVariables::OMEGA>())
        {
          Log::error() << "Reeca: omega signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::QEXT>()
            && !signals_.template isLinked<ReecaExternalVariables::QEXT>())
        {
          Log::error() << "Reeca: qext signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::PFAREF>()
            && !signals_.template isLinked<ReecaExternalVariables::PFAREF>())
        {
          Log::error() << "Reeca: pfaref signal attached with no linked source\n";
          ret += 1;
        }

        if (signals_.template isAttached<ReecaExternalVariables::PREF>()
            && !signals_.template isLinked<ReecaExternalVariables::PREF>())
        {
          Log::error() << "Reeca: pref signal attached with no linked source\n";
          ret += 1;
        }

        return ret;
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::initialize()
      {
        if (parameter_error_count_ > 0 || verify() > 0)
        {
          Log::error() << "Reeca: cannot initialize with invalid configuration\n";
          return 1;
        }

        const auto VMEAS     = static_cast<size_t>(ReecaInternalVariables::VMEAS);
        const auto PMEAS     = static_cast<size_t>(ReecaInternalVariables::PMEAS);
        const auto XPIQ      = static_cast<size_t>(ReecaInternalVariables::XPIQ);
        const auto XPIV      = static_cast<size_t>(ReecaInternalVariables::XPIV);
        const auto QV        = static_cast<size_t>(ReecaInternalVariables::QV);
        const auto PORD      = static_cast<size_t>(ReecaInternalVariables::PORD);
        const auto VT        = static_cast<size_t>(ReecaInternalVariables::VT);
        const auto VMEASSAFE = static_cast<size_t>(ReecaInternalVariables::VMEASSAFE);
        const auto SDIP      = static_cast<size_t>(ReecaInternalVariables::SDIP);
        const auto VERR      = static_cast<size_t>(ReecaInternalVariables::VERR);
        const auto IQV       = static_cast<size_t>(ReecaInternalVariables::IQV);
        const auto QREF      = static_cast<size_t>(ReecaInternalVariables::QREF);
        const auto EQ        = static_cast<size_t>(ReecaInternalVariables::EQ);
        const auto VPIQ      = static_cast<size_t>(ReecaInternalVariables::VPIQ);
        const auto EPIV      = static_cast<size_t>(ReecaInternalVariables::EPIV);
        const auto FPORD     = static_cast<size_t>(ReecaInternalVariables::FPORD);
        const auto RPORD     = static_cast<size_t>(ReecaInternalVariables::RPORD);
        const auto IQCIRC    = static_cast<size_t>(ReecaInternalVariables::IQCIRC);
        const auto IPCIRC    = static_cast<size_t>(ReecaInternalVariables::IPCIRC);
        const auto IQMAX     = static_cast<size_t>(ReecaInternalVariables::IQMAX);
        const auto IPMAX     = static_cast<size_t>(ReecaInternalVariables::IPMAX);
        const auto IQBASE    = static_cast<size_t>(ReecaInternalVariables::IQBASE);
        const auto IQRAW     = static_cast<size_t>(ReecaInternalVariables::IQRAW);
        const auto IQCMD     = static_cast<size_t>(ReecaInternalVariables::IQCMD);
        const auto IPCMD     = static_cast<size_t>(ReecaInternalVariables::IPCMD);

        const ScalarT vr = Vr();
        const ScalarT vi = Vi();

        y_[VT] = std::sqrt(vr * vr + vi * vi);

        const RealT vt0 = static_cast<RealT>(y_[VT]);
        if (vt0 <= ZERO<RealT>)
        {
          Log::error() << "Reeca: terminal voltage magnitude must be positive at initialization\n";
          return 1;
        }

        if (!(Vdip_ < vt0 && vt0 < Vup_))
        {
          Log::error() << "Reeca: standard initialization requires Vdip < Vt0 < Vup\n";
          return 1;
        }

        if (!Vref0_given_)
        {
          Vref0_ = vt0;
        }

        y_[VMEAS]     = y_[VT];
        y_[VMEASSAFE] = Math::max(y_[VMEAS], static_cast<RealT>(0.01));

        const ScalarT ipcmd0 = y_[IPCMD];
        const ScalarT iqcmd0 = y_[IQCMD];
        const ScalarT pe0    = ipcmd0 * y_[VMEASSAFE];
        const ScalarT qgen0  = iqcmd0 * y_[VMEASSAFE];

        ScalarT omega0{ZERO<RealT>};
        if (signals_.template isAttached<ReecaExternalVariables::OMEGA>())
        {
          omega0 = signals_.template readExternalVariable<ReecaExternalVariables::OMEGA>();
        }

        const ScalarT pref_denominator = ONE<RealT> + PFlag_ * omega0;
        if (static_cast<RealT>(pref_denominator) == ZERO<RealT>)
        {
          Log::error() << "Reeca: pref initialization denominator is zero\n";
          return 1;
        }

        const ScalarT qref0 = qgen0;
        const ScalarT pref0 = pe0 / pref_denominator;
        ScalarT       pfaref0{ZERO<RealT>};
        if (std::abs(static_cast<RealT>(pe0)) > ZERO<RealT>)
        {
          pfaref0 = static_cast<ScalarT>(std::atan(static_cast<RealT>(qref0 / pe0)));
        }

        pe_set_     = toSystemBase(pe0);
        qgen_set_   = toSystemBase(qgen0);
        omega_set_  = omega0;
        qext_set_   = qref0;
        pfaref_set_ = pfaref0;
        pref_set_   = pref0;

        if (signals_.template isAttached<ReecaExternalVariables::QEXT>())
        {
          signals_.template writeExternalVariable<ReecaExternalVariables::QEXT>(qref0);
        }
        if (signals_.template isAttached<ReecaExternalVariables::PFAREF>())
        {
          signals_.template writeExternalVariable<ReecaExternalVariables::PFAREF>(pfaref0);
        }
        if (signals_.template isAttached<ReecaExternalVariables::PREF>())
        {
          signals_.template writeExternalVariable<ReecaExternalVariables::PREF>(pref0);
        }

        y_[PMEAS]              = pe0;
        const ScalarT qref_pf  = PfFlag_ * y_[PMEAS] * std::tan(pfaref_set_);
        const ScalarT qref_ext = pf_off_ * qext_set_;

        y_[SDIP] = Math::outside(y_[VT], Vdip_, Vup_);
        y_[VERR] = Math::deadband2(Vref0_ - y_[VMEAS], dbd1_, dbd2_);
        y_[IQV]  = Math::clamp(kqv_ * y_[VERR], Iql1_, Iqh1_);
        y_[QREF] = qref_pf + qref_ext;

        const RealT qref0_value = static_cast<RealT>(y_[QREF]);
        if (qref0_value < Qmin_ || qref0_value > Qmax_)
        {
          Log::error() << "Reeca: standard initialization requires Qref0 within Qmin/Qmax\n";
          return 1;
        }

        y_[EQ] = Math::clamp(y_[QREF], Qmin_, Qmax_) - qgen0;
        y_[QV] = y_[QREF] / y_[VMEASSAFE];

        const ScalarT pord0       = pref_denominator * pref0;
        const RealT   pord0_value = static_cast<RealT>(pord0);
        if (pord0_value < Pmin_ || pord0_value > Pmax_)
        {
          Log::error() << "Reeca: standard initialization requires Pord0 within Pmin/Pmax\n";
          return 1;
        }

        y_[PORD]  = pord0;
        y_[FPORD] = ZERO<RealT>;
        y_[RPORD] = ZERO<RealT>;

        static constexpr RealT kSmallTol  = static_cast<RealT>(1.0e-9);
        static constexpr RealT kSatMargin = static_cast<RealT>(0.1);

        auto steadyPiOutput = [&](const ScalarT interior_target,
                                  const ScalarT error,
                                  const ScalarT integral_rate,
                                  const ScalarT lower,
                                  const ScalarT upper) -> ScalarT
        {
          const RealT error_value = static_cast<RealT>(error);
          const RealT rate_value  = static_cast<RealT>(integral_rate);

          if (std::abs(error_value) <= kSmallTol || std::abs(rate_value) <= kSmallTol)
          {
            return interior_target;
          }

          if (rate_value > ZERO<RealT>)
          {
            return upper + static_cast<ScalarT>(kSatMargin);
          }

          return lower - static_cast<ScalarT>(kSatMargin);
        };

        const ScalarT xpiq_rate   = Kqi_ * y_[EQ];
        const ScalarT vpiq_target = VFlag_ * y_[VMEAS] + v_off_ * y_[QREF];
        const ScalarT vpiq_arg    = steadyPiOutput(vpiq_target,
                                                y_[EQ],
                                                xpiq_rate,
                                                static_cast<ScalarT>(Vmin_),
                                                static_cast<ScalarT>(Vmax_));
        y_[VPIQ]                  = Math::clamp(vpiq_arg, Vmin_, Vmax_);
        y_[EPIV]                  = VFlag_ * y_[VPIQ] + v_off_ * (y_[QREF] + Vref1_) - y_[VMEAS];
        y_[XPIQ]                  = vpiq_arg - Kqp_ * y_[EQ];

        const ScalarT gq0 = static_cast<ScalarT>(Iq1_)
                            + Math::linseg(y_[VMEAS], Vq1_, Vq2_, Iq2_ - Iq1_)
                            + Math::linseg(y_[VMEAS], Vq2_, Vq3_, Iq3_ - Iq2_)
                            + Math::linseg(y_[VMEAS], Vq3_, Vq4_, Iq4_ - Iq3_);
        const ScalarT gp0 = static_cast<ScalarT>(Ip1_)
                            + Math::linseg(y_[VMEAS], Vp1_, Vp2_, Ip2_ - Ip1_)
                            + Math::linseg(y_[VMEAS], Vp2_, Vp3_, Ip3_ - Ip2_)
                            + Math::linseg(y_[VMEAS], Vp3_, Vp4_, Ip4_ - Ip3_);

        const ScalarT iqbase_target = qgen0 / y_[VMEASSAFE];
        const ScalarT xpiv_rate     = Kvi_ * y_[EPIV];
        const ScalarT ip_star       = y_[PORD] / y_[VMEASSAFE];

        auto initializeReactiveBase = [&]()
        {
          const ScalarT piv_lower = -y_[IQMAX];
          const ScalarT piv_upper = y_[IQMAX];
          const ScalarT piv_arg   = steadyPiOutput(iqbase_target,
                                                 y_[EPIV],
                                                 xpiv_rate,
                                                 piv_lower,
                                                 piv_upper);
          y_[IQBASE]              = Math::clamp(piv_arg, -y_[IQMAX], y_[IQMAX]);
          y_[XPIV]                = piv_arg - Kvp_ * y_[EPIV];
        };

        auto initializeReactiveCommand = [&]()
        {
          y_[IQRAW] = QFlag_ * y_[IQBASE] + q_off_ * y_[QV] + y_[SDIP] * y_[IQV];
          y_[IQCMD] = Math::clamp(y_[IQRAW], -y_[IQMAX], y_[IQMAX]);
        };

        auto rejectOutsideReactiveLimits = [&](const ScalarT value, const char* message)
        {
          const RealT limit = static_cast<RealT>(y_[IQMAX]);
          if (static_cast<RealT>(value) < -limit || static_cast<RealT>(value) > limit)
          {
            Log::error() << "Reeca: " << message << '\n';
            return true;
          }
          return false;
        };

        auto rejectOutsideActiveLimits = [&](const ScalarT value, const char* message)
        {
          const RealT limit = static_cast<RealT>(y_[IPMAX]);
          if (static_cast<RealT>(value) < ZERO<RealT> || static_cast<RealT>(value) > limit)
          {
            Log::error() << "Reeca: " << message << '\n';
            return true;
          }
          return false;
        };

        if (Pqflag_ == ZERO<RealT>)
        {
          y_[IQCIRC] = Imax_;
          y_[IQMAX]  = Math::min(gq0, y_[IQCIRC]);
          if (rejectOutsideReactiveLimits(
                  iqbase_target,
                  "standard initialization requires Iq0 within reactive-current limits"))
          {
            return 1;
          }
          initializeReactiveBase();
          initializeReactiveCommand();
          if (rejectOutsideReactiveLimits(
                  y_[IQRAW],
                  "standard initialization requires raw Iq0 within reactive-current limits"))
          {
            return 1;
          }

          const ScalarT ip_radicand = Imax_ * Imax_ - y_[IQCMD] * y_[IQCMD];
          if (static_cast<RealT>(ip_radicand) < ZERO<RealT>)
          {
            Log::error() << "Reeca: initial active-current circle radicand is negative\n";
            return 1;
          }
          y_[IPCIRC] = std::sqrt(ip_radicand);
          y_[IPMAX]  = Math::min(gp0, y_[IPCIRC]);
          if (rejectOutsideActiveLimits(
                  ip_star,
                  "standard initialization requires Ip0 within active-current limits"))
          {
            return 1;
          }
          y_[IPCMD] = Math::clamp(ip_star, ZERO<RealT>, y_[IPMAX]);
        }
        else
        {
          y_[IPCIRC] = Imax_;
          y_[IPMAX]  = Math::min(gp0, y_[IPCIRC]);
          if (rejectOutsideActiveLimits(
                  ip_star,
                  "standard initialization requires Ip0 within active-current limits"))
          {
            return 1;
          }
          y_[IPCMD] = Math::clamp(ip_star, ZERO<RealT>, y_[IPMAX]);

          const ScalarT iq_radicand = Imax_ * Imax_ - y_[IPCMD] * y_[IPCMD];
          if (static_cast<RealT>(iq_radicand) < ZERO<RealT>)
          {
            Log::error() << "Reeca: initial reactive-current circle radicand is negative\n";
            return 1;
          }
          y_[IQCIRC] = std::sqrt(iq_radicand);
          y_[IQMAX]  = Math::min(gq0, y_[IQCIRC]);
          if (rejectOutsideReactiveLimits(
                  iqbase_target,
                  "standard initialization requires Iq0 within reactive-current limits"))
          {
            return 1;
          }
          initializeReactiveBase();
          initializeReactiveCommand();
          if (rejectOutsideReactiveLimits(
                  y_[IQRAW],
                  "standard initialization requires raw Iq0 within reactive-current limits"))
          {
            return 1;
          }
        }

        std::fill(yp_.begin(), yp_.end(), ZERO<RealT>);

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::tagDifferentiable()
      {
        std::fill(tag_.begin(), tag_.end(), false);
        tag_[static_cast<size_t>(ReecaInternalVariables::VMEAS)] = (Trv_ > ZERO<RealT>);
        tag_[static_cast<size_t>(ReecaInternalVariables::PMEAS)] = (Tp_ > ZERO<RealT>);
        tag_[static_cast<size_t>(ReecaInternalVariables::XPIQ)]  = true;
        tag_[static_cast<size_t>(ReecaInternalVariables::XPIV)]  = true;
        tag_[static_cast<size_t>(ReecaInternalVariables::QV)]    = true;
        tag_[static_cast<size_t>(ReecaInternalVariables::PORD)]  = true;
        return 0;
      }

      template <typename scalar_type, typename index_type>
      __attribute__((always_inline)) inline int
      Reeca<scalar_type, index_type>::evaluateInternalResidual(
          ScalarT* y,
          ScalarT* yp,
          ScalarT* wb,
          ScalarT* ws,
          ScalarT* f)
      {
        const auto VMEAS     = static_cast<size_t>(ReecaInternalVariables::VMEAS);
        const auto PMEAS     = static_cast<size_t>(ReecaInternalVariables::PMEAS);
        const auto XPIQ      = static_cast<size_t>(ReecaInternalVariables::XPIQ);
        const auto XPIV      = static_cast<size_t>(ReecaInternalVariables::XPIV);
        const auto QV        = static_cast<size_t>(ReecaInternalVariables::QV);
        const auto PORD      = static_cast<size_t>(ReecaInternalVariables::PORD);
        const auto VT        = static_cast<size_t>(ReecaInternalVariables::VT);
        const auto VMEASSAFE = static_cast<size_t>(ReecaInternalVariables::VMEASSAFE);
        const auto SDIP      = static_cast<size_t>(ReecaInternalVariables::SDIP);
        const auto VERR      = static_cast<size_t>(ReecaInternalVariables::VERR);
        const auto IQV       = static_cast<size_t>(ReecaInternalVariables::IQV);
        const auto QREF      = static_cast<size_t>(ReecaInternalVariables::QREF);
        const auto EQ        = static_cast<size_t>(ReecaInternalVariables::EQ);
        const auto VPIQ      = static_cast<size_t>(ReecaInternalVariables::VPIQ);
        const auto EPIV      = static_cast<size_t>(ReecaInternalVariables::EPIV);
        const auto FPORD     = static_cast<size_t>(ReecaInternalVariables::FPORD);
        const auto RPORD     = static_cast<size_t>(ReecaInternalVariables::RPORD);
        const auto IQCIRC    = static_cast<size_t>(ReecaInternalVariables::IQCIRC);
        const auto IPCIRC    = static_cast<size_t>(ReecaInternalVariables::IPCIRC);
        const auto IQMAX     = static_cast<size_t>(ReecaInternalVariables::IQMAX);
        const auto IPMAX     = static_cast<size_t>(ReecaInternalVariables::IPMAX);
        const auto IQBASE    = static_cast<size_t>(ReecaInternalVariables::IQBASE);
        const auto IQRAW     = static_cast<size_t>(ReecaInternalVariables::IQRAW);
        const auto IQCMD     = static_cast<size_t>(ReecaInternalVariables::IQCMD);
        const auto IPCMD     = static_cast<size_t>(ReecaInternalVariables::IPCMD);

        const auto PE     = static_cast<size_t>(ReecaExternalVariables::PE);
        const auto QGEN   = static_cast<size_t>(ReecaExternalVariables::QGEN);
        const auto OMEGA  = static_cast<size_t>(ReecaExternalVariables::OMEGA);
        const auto QEXT   = static_cast<size_t>(ReecaExternalVariables::QEXT);
        const auto PFAREF = static_cast<size_t>(ReecaExternalVariables::PFAREF);
        const auto PREF   = static_cast<size_t>(ReecaExternalVariables::PREF);

        const ScalarT vr = wb[0];
        const ScalarT vi = wb[1];

        const ScalarT pe     = toComponentBase(ws[PE]);
        const ScalarT qgen   = toComponentBase(ws[QGEN]);
        const ScalarT omega  = ws[OMEGA];
        const ScalarT qext   = ws[QEXT];
        const ScalarT pfaref = ws[PFAREF];
        const ScalarT pref   = ws[PREF];

        const ScalarT not_sdip          = ONE<RealT> - y[SDIP];
        const ScalarT xpiq_arg          = Kqp_ * y[EQ] + y[XPIQ];
        const ScalarT xpiv_arg          = Kvp_ * y[EPIV] + y[XPIV];
        const ScalarT xpiq_limited_rate = Math::antiwindup(xpiq_arg, Kqi_ * y[EQ], Vmin_, Vmax_);
        const ScalarT xpiv_limited_rate =
            Math::antiwindup(xpiv_arg, Kvi_ * y[EPIV], -y[IQMAX], y[IQMAX]);
        const ScalarT pord_limited_rate = Math::antiwindup(y[PORD], y[RPORD], Pmin_, Pmax_);
        const ScalarT qref_pf           = PfFlag_ * y[PMEAS] * std::tan(pfaref);
        const ScalarT qref_ext          = pf_off_ * qext;
        const ScalarT epiv_target       = VFlag_ * y[VPIQ] + v_off_ * (y[QREF] + Vref1_);
        const ScalarT iqcirc_current_sq = Pqflag_ * y[IPCMD] * y[IPCMD];
        const ScalarT ipcirc_current_sq = (ONE<RealT> - Pqflag_) * y[IQCMD] * y[IQCMD];
        const ScalarT gq                = static_cast<ScalarT>(Iq1_)
                           + Math::linseg(y[VMEAS], Vq1_, Vq2_, Iq2_ - Iq1_)
                           + Math::linseg(y[VMEAS], Vq2_, Vq3_, Iq3_ - Iq2_)
                           + Math::linseg(y[VMEAS], Vq3_, Vq4_, Iq4_ - Iq3_);
        const ScalarT gp = static_cast<ScalarT>(Ip1_)
                           + Math::linseg(y[VMEAS], Vp1_, Vp2_, Ip2_ - Ip1_)
                           + Math::linseg(y[VMEAS], Vp2_, Vp3_, Ip3_ - Ip2_)
                           + Math::linseg(y[VMEAS], Vp3_, Vp4_, Ip4_ - Ip3_);
        const ScalarT iqraw_target = QFlag_ * y[IQBASE] + q_off_ * y[QV] + y[SDIP] * y[IQV];

        f[VMEAS] = -Trv_ * yp[VMEAS] - y[VMEAS] + y[VT];
        f[PMEAS] = -Tp_ * yp[PMEAS] - y[PMEAS] + pe;
        f[XPIQ]  = -yp[XPIQ] + not_sdip * xpiq_limited_rate;
        f[XPIV]  = -yp[XPIV] + not_sdip * xpiv_limited_rate;
        f[QV]    = -Tiq_ * yp[QV] - not_sdip * y[QV] + not_sdip * y[QREF] / y[VMEASSAFE];
        f[PORD]  = -yp[PORD] + not_sdip * pord_limited_rate;

        f[VT]        = -y[VT] * y[VT] + vr * vr + vi * vi;
        f[VMEASSAFE] = -y[VMEASSAFE] + Math::max(y[VMEAS], static_cast<RealT>(0.01));
        f[SDIP]      = -y[SDIP] + Math::outside(y[VT], Vdip_, Vup_);
        f[VERR]      = -y[VERR] + Math::deadband2(Vref0_ - y[VMEAS], dbd1_, dbd2_);
        f[IQV]       = -y[IQV] + Math::clamp(kqv_ * y[VERR], Iql1_, Iqh1_);
        f[QREF]      = -y[QREF] + qref_pf + qref_ext;
        f[EQ]        = -y[EQ] + Math::clamp(y[QREF], Qmin_, Qmax_) - qgen;
        f[VPIQ]      = -y[VPIQ] + Math::clamp(xpiq_arg, Vmin_, Vmax_);
        f[EPIV]      = -y[EPIV] + epiv_target - y[VMEAS];
        f[FPORD]     = -Tpord_ * y[FPORD] + (ONE<RealT> + PFlag_ * omega) * pref - y[PORD];
        f[RPORD]     = -y[RPORD] + Math::clamp(y[FPORD], dPmin_, dPmax_);

        f[IQCIRC] = -y[IQCIRC] * y[IQCIRC] + Imax_ * Imax_ - iqcirc_current_sq;
        f[IPCIRC] = -y[IPCIRC] * y[IPCIRC] + Imax_ * Imax_ - ipcirc_current_sq;
        f[IQMAX]  = -y[IQMAX] + Math::min(gq, y[IQCIRC]);
        f[IPMAX]  = -y[IPMAX] + Math::min(gp, y[IPCIRC]);
        f[IQBASE] = -y[IQBASE] + Math::clamp(xpiv_arg, -y[IQMAX], y[IQMAX]);
        f[IQRAW]  = -y[IQRAW] + iqraw_target;
        f[IQCMD]  = -y[IQCMD] + Math::clamp(y[IQRAW], -y[IQMAX], y[IQMAX]);
        f[IPCMD]  = -y[IPCMD] + Math::clamp(y[PORD] / y[VMEASSAFE], ZERO<RealT>, y[IPMAX]);

        return 0;
      }

      template <typename scalar_type, typename index_type>
      int Reeca<scalar_type, index_type>::evaluateResidual()
      {
        const auto PE     = static_cast<size_t>(ReecaExternalVariables::PE);
        const auto QGEN   = static_cast<size_t>(ReecaExternalVariables::QGEN);
        const auto OMEGA  = static_cast<size_t>(ReecaExternalVariables::OMEGA);
        const auto QEXT   = static_cast<size_t>(ReecaExternalVariables::QEXT);
        const auto PFAREF = static_cast<size_t>(ReecaExternalVariables::PFAREF);
        const auto PREF   = static_cast<size_t>(ReecaExternalVariables::PREF);

        ws_[PE]     = pe_set_;
        ws_[QGEN]   = qgen_set_;
        ws_[OMEGA]  = omega_set_;
        ws_[QEXT]   = qext_set_;
        ws_[PFAREF] = pfaref_set_;
        ws_[PREF]   = pref_set_;
        std::fill(ws_indices_.begin(), ws_indices_.end(), INVALID_INDEX<IdxT>);

        if (signals_.template isAttached<ReecaExternalVariables::PE>())
        {
          ws_[PE] = signals_.template readExternalVariable<ReecaExternalVariables::PE>();
          ws_indices_[PE] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::PE>();
        }
        if (signals_.template isAttached<ReecaExternalVariables::QGEN>())
        {
          ws_[QGEN] = signals_.template readExternalVariable<ReecaExternalVariables::QGEN>();
          ws_indices_[QGEN] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::QGEN>();
        }
        if (signals_.template isAttached<ReecaExternalVariables::OMEGA>())
        {
          ws_[OMEGA] = signals_.template readExternalVariable<ReecaExternalVariables::OMEGA>();
          ws_indices_[OMEGA] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::OMEGA>();
        }
        if (signals_.template isAttached<ReecaExternalVariables::QEXT>())
        {
          ws_[QEXT] = signals_.template readExternalVariable<ReecaExternalVariables::QEXT>();
          ws_indices_[QEXT] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::QEXT>();
        }
        if (signals_.template isAttached<ReecaExternalVariables::PFAREF>())
        {
          ws_[PFAREF] = signals_.template readExternalVariable<ReecaExternalVariables::PFAREF>();
          ws_indices_[PFAREF] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::PFAREF>();
        }
        if (signals_.template isAttached<ReecaExternalVariables::PREF>())
        {
          ws_[PREF] = signals_.template readExternalVariable<ReecaExternalVariables::PREF>();
          ws_indices_[PREF] =
              signals_.template readExternalVariableIndex<ReecaExternalVariables::PREF>();
        }

        wb_[0] = Vr();
        wb_[1] = Vi();

        evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), f_.data());
        return 0;
      }
    } // namespace Converter
  } // namespace PhasorDynamics
} // namespace GridKit
