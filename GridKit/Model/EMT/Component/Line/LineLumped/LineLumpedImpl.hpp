#pragma once

#include <cmath>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumped.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    LineLumped<scalar_type, index_type>::LineLumped(BusT*             bus1,
                                                    BusT*             bus2,
                                                    const ModelDataT& data)
      : bus1_(bus1),
        bus2_(bus2),
        monitor_(std::make_unique<MonitorT>(data))
    {
      size_ = 9;
      initializeParameters(data);
      initializeMonitor();
    }

    template <typename scalar_type, typename index_type>
    LineLumped<scalar_type, index_type>::~LineLumped() = default;

    template <typename scalar_type, typename index_type>
    void LineLumped<scalar_type, index_type>::initializeParameters(const ModelDataT& data)
    {
      using Parameter = typename ModelDataT::Parameters;
      if (data.parameters.contains(Parameter::dx))
      {
        dx_ = std::get<RealT>(data.parameters.at(Parameter::dx));
      }
      if (data.parameters.contains(Parameter::Rp))
      {
        Rp_ = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameter::Rp));
      }
      if (data.parameters.contains(Parameter::Lp))
      {
        Lp_ = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameter::Lp));
      }
      if (data.parameters.contains(Parameter::Gp))
      {
        Gp_ = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameter::Gp));
      }
      if (data.parameters.contains(Parameter::Cp))
      {
        Cp_ = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameter::Cp));
      }

      R_ = {{{dx_ * Rp_[0][0], dx_ * Rp_[0][1], dx_ * Rp_[0][2]},
             {dx_ * Rp_[1][0], dx_ * Rp_[1][1], dx_ * Rp_[1][2]},
             {dx_ * Rp_[2][0], dx_ * Rp_[2][1], dx_ * Rp_[2][2]}}};
      L_ = {{{dx_ * Lp_[0][0], dx_ * Lp_[0][1], dx_ * Lp_[0][2]},
             {dx_ * Lp_[1][0], dx_ * Lp_[1][1], dx_ * Lp_[1][2]},
             {dx_ * Lp_[2][0], dx_ * Lp_[2][1], dx_ * Lp_[2][2]}}};
      G_ = {{{dx_ * Gp_[0][0], dx_ * Gp_[0][1], dx_ * Gp_[0][2]},
             {dx_ * Gp_[1][0], dx_ * Gp_[1][1], dx_ * Gp_[1][2]},
             {dx_ * Gp_[2][0], dx_ * Gp_[2][1], dx_ * Gp_[2][2]}}};
      C_ = {{{dx_ * Cp_[0][0], dx_ * Cp_[0][1], dx_ * Cp_[0][2]},
             {dx_ * Cp_[1][0], dx_ * Cp_[1][1], dx_ * Cp_[1][2]},
             {dx_ * Cp_[2][0], dx_ * Cp_[2][1], dx_ * Cp_[2][2]}}};
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::setGridKitComponentID(IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void LineLumped<scalar_type, index_type>::appendBusVoltageContributions(
        std::vector<BusVoltageContribution<ScalarT, IdxT>>& contributions) const
    {
      if (bus1_ != nullptr)
      {
        contributions.push_back({bus1_->busID(), C_, G_});
      }
      if (bus2_ != nullptr)
      {
        contributions.push_back({bus2_->busID(), C_, G_});
      }
    }

    template <typename scalar_type, typename index_type>
    void LineLumped<scalar_type, index_type>::appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const
    {
      variables.push_back({"i12", std::nullopt, 0});
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::allocate()
    {
      if (!allocated_)
      {
        this->allocateVectors(size_);
      }
      tag_.resize(9);
      variable_indices_.resize(9);
      residual_indices_.resize(9);
      for (IdxT index = 0; index < 9; ++index)
      {
        this->setVariableIndex(index, index);
        this->setResidualIndex(index, index);
      }
      wb_.resize(6);
      wbp_.resize(6);
      h_.resize(6);
      bus_variable_indices_.resize(6);
      allocated_ = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::verify() const
    {
      int status = 0;
      if (bus1_ == nullptr || bus2_ == nullptr || bus1_ == bus2_)
      {
        Log::error() << "EMT::LineLumped: two distinct, non-null buses are required\n";
        ++status;
      }
      if (!(dx_ > RealT{0.0}) || !std::isfinite(dx_))
      {
        Log::error() << "EMT::LineLumped: dx must be finite and positive\n";
        ++status;
      }
      if (!Detail::positiveSemidefinite(Rp_))
      {
        Log::error() << "EMT::LineLumped: Rp must be finite, symmetric, and positive semidefinite\n";
        ++status;
      }
      if (!Detail::positiveDefinite(Lp_))
      {
        Log::error() << "EMT::LineLumped: Lp must be finite, symmetric, and positive definite\n";
        ++status;
      }
      if (!Detail::positiveSemidefinite(Gp_))
      {
        Log::error() << "EMT::LineLumped: Gp must be finite, symmetric, and positive semidefinite\n";
        ++status;
      }
      if (!Detail::positiveSemidefinite(Cp_))
      {
        Log::error() << "EMT::LineLumped: Cp must be finite, symmetric, and positive semidefinite\n";
        ++status;
      }
      return status;
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::initialize()
    {
      std::fill_n(y_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      std::fill_n(yp_.getData(), static_cast<std::size_t>(size_), ScalarT{0.0});
      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::tagDifferentiable()
    {
      tag_[0] = true;
      tag_[1] = true;
      tag_[2] = true;
      tag_[3] = false;
      tag_[4] = false;
      tag_[5] = false;
      tag_[6] = false;
      tag_[7] = false;
      tag_[8] = false;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::setAbsoluteTolerance(RealT absolute_tolerance)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(absolute_tolerance));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int LineLumped<scalar_type, index_type>::evaluateInternalResidual(
        const ScalarT* y,
        const ScalarT* yp,
        const ScalarT* wb,
        const ScalarT* wbp,
        ScalarT*       f)
    {
      const ScalarT i12a   = y[0];
      const ScalarT i12b   = y[1];
      const ScalarT i12c   = y[2];
      const ScalarT i12ap  = yp[0];
      const ScalarT i12bp  = yp[1];
      const ScalarT i12cp  = yp[2];
      const ScalarT i_sh1a = y[3];
      const ScalarT i_sh1b = y[4];
      const ScalarT i_sh1c = y[5];
      const ScalarT i_sh2a = y[6];
      const ScalarT i_sh2b = y[7];
      const ScalarT i_sh2c = y[8];

      const ScalarT v1a  = wb[0];
      const ScalarT v1b  = wb[1];
      const ScalarT v1c  = wb[2];
      const ScalarT v2a  = wb[3];
      const ScalarT v2b  = wb[4];
      const ScalarT v2c  = wb[5];
      const ScalarT v1ap = wbp[0];
      const ScalarT v1bp = wbp[1];
      const ScalarT v1cp = wbp[2];
      const ScalarT v2ap = wbp[3];
      const ScalarT v2bp = wbp[4];
      const ScalarT v2cp = wbp[5];

      f[0] = R_[0][0] * i12a + R_[0][1] * i12b + R_[0][2] * i12c
             + L_[0][0] * i12ap + L_[0][1] * i12bp + L_[0][2] * i12cp + v2a - v1a;
      f[1] = R_[1][0] * i12a + R_[1][1] * i12b + R_[1][2] * i12c
             + L_[1][0] * i12ap + L_[1][1] * i12bp + L_[1][2] * i12cp + v2b - v1b;
      f[2] = R_[2][0] * i12a + R_[2][1] * i12b + R_[2][2] * i12c
             + L_[2][0] * i12ap + L_[2][1] * i12bp + L_[2][2] * i12cp + v2c - v1c;

      f[3] = G_[0][0] * v1a + G_[0][1] * v1b + G_[0][2] * v1c
             + C_[0][0] * v1ap + C_[0][1] * v1bp + C_[0][2] * v1cp + RealT{2.0} * i_sh1a;
      f[4] = G_[1][0] * v1a + G_[1][1] * v1b + G_[1][2] * v1c
             + C_[1][0] * v1ap + C_[1][1] * v1bp + C_[1][2] * v1cp + RealT{2.0} * i_sh1b;
      f[5] = G_[2][0] * v1a + G_[2][1] * v1b + G_[2][2] * v1c
             + C_[2][0] * v1ap + C_[2][1] * v1bp + C_[2][2] * v1cp + RealT{2.0} * i_sh1c;

      f[6] = G_[0][0] * v2a + G_[0][1] * v2b + G_[0][2] * v2c
             + C_[0][0] * v2ap + C_[0][1] * v2bp + C_[0][2] * v2cp + RealT{2.0} * i_sh2a;
      f[7] = G_[1][0] * v2a + G_[1][1] * v2b + G_[1][2] * v2c
             + C_[1][0] * v2ap + C_[1][1] * v2bp + C_[1][2] * v2cp + RealT{2.0} * i_sh2b;
      f[8] = G_[2][0] * v2a + G_[2][1] * v2b + G_[2][2] * v2c
             + C_[2][0] * v2ap + C_[2][1] * v2bp + C_[2][2] * v2cp + RealT{2.0} * i_sh2c;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int LineLumped<scalar_type, index_type>::evaluateBus1Residual(
        const ScalarT* y,
        ScalarT*       h)
    {
      h[0] = y[3] - y[0];
      h[1] = y[4] - y[1];
      h[2] = y[5] - y[2];
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int LineLumped<scalar_type, index_type>::evaluateBus2Residual(
        const ScalarT* y,
        ScalarT*       h)
    {
      h[0] = y[6] + y[0];
      h[1] = y[7] + y[1];
      h[2] = y[8] + y[2];
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::evaluateResidual()
    {
      wb_[0]  = bus1_->Va();
      wb_[1]  = bus1_->Vb();
      wb_[2]  = bus1_->Vc();
      wb_[3]  = bus2_->Va();
      wb_[4]  = bus2_->Vb();
      wb_[5]  = bus2_->Vc();
      wbp_[0] = bus1_->Vap();
      wbp_[1] = bus1_->Vbp();
      wbp_[2] = bus1_->Vcp();
      wbp_[3] = bus2_->Vap();
      wbp_[4] = bus2_->Vbp();
      wbp_[5] = bus2_->Vcp();

      const auto* y  = y_.getData();
      const auto* yp = yp_.getData();
      auto*       f  = f_.getData();
      evaluateInternalResidual(y, yp, wb_.data(), wbp_.data(), f);
      if (bus1_->differentiatedKCL())
      {
        h_[0] = -yp[0];
        h_[1] = -yp[1];
        h_[2] = -yp[2];
      }
      else
      {
        evaluateBus1Residual(y, h_.data());
      }
      if (bus2_->differentiatedKCL())
      {
        h_[3] = yp[0];
        h_[4] = yp[1];
        h_[5] = yp[2];
      }
      else
      {
        evaluateBus2Residual(y, h_.data() + 3);
      }

      bus1_->accumulateCurrent(h_[0], h_[1], h_[2]);
      bus2_->accumulateCurrent(h_[3], h_[4], h_[5]);
      f_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    const Model::VariableMonitorBase* LineLumped<scalar_type, index_type>::getMonitor() const
    {
      return monitor_.get();
    }

    template <typename scalar_type, typename index_type>
    void LineLumped<scalar_type, index_type>::initializeMonitor()
    {
      using Variable = typename ModelDataT::MonitorableVariables;
      monitor_->set(Variable::i12a, [this]
                    { return y_.getData()[0]; });
      monitor_->set(Variable::i12b, [this]
                    { return y_.getData()[1]; });
      monitor_->set(Variable::i12c, [this]
                    { return y_.getData()[2]; });
      monitor_->set(Variable::i_sh1a, [this]
                    { return y_.getData()[3]; });
      monitor_->set(Variable::i_sh1b, [this]
                    { return y_.getData()[4]; });
      monitor_->set(Variable::i_sh1c, [this]
                    { return y_.getData()[5]; });
      monitor_->set(Variable::i_sh2a, [this]
                    { return y_.getData()[6]; });
      monitor_->set(Variable::i_sh2b, [this]
                    { return y_.getData()[7]; });
      monitor_->set(Variable::i_sh2c, [this]
                    { return y_.getData()[8]; });
    }
  } // namespace EMT
} // namespace GridKit
