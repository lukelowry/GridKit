#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumped.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    LineLumped<scalar_type, index_type, N>::LineLumped(BusT* bus1, BusT* bus2)
      : LineLumped(bus1, bus2, ModelDataT{})
    {
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    LineLumped<scalar_type, index_type, N>::LineLumped(BusT* bus1, BusT* bus2, const ModelDataT& data)
      : bus1_(bus1),
        bus2_(bus2),
        data_(data),
        series_z_(scaleVectorFitData(data_.Zp, data_.dx)),
        shunt_y1_(scaleVectorFitData(data_.Yp, RealT{0.5} * data_.dx)),
        shunt_y2_(scaleVectorFitData(data_.Yp, RealT{0.5} * data_.dx))
    {
      setLayout();
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    typename LineLumped<scalar_type, index_type, N>::ModelDataT::VectorFitDataT
    LineLumped<scalar_type, index_type, N>::scaleVectorFitData(
        const typename ModelDataT::VectorFitDataT& data,
        RealT                                      scale)
    {
      auto scaled = data;

      for (std::size_t r = 0; r < N; ++r)
      {
        for (std::size_t c = 0; c < N; ++c)
        {
          scaled.D(r, c) *= scale;
          scaled.E(r, c) *= scale;
        }
      }

      for (auto& A : scaled.A)
      {
        for (std::size_t r = 0; r < N; ++r)
        {
          for (std::size_t c = 0; c < N; ++c)
          {
            A(r, c) *= scale;
          }
        }
      }

      for (auto& B : scaled.B)
      {
        for (std::size_t r = 0; r < N; ++r)
        {
          for (std::size_t c = 0; c < N; ++c)
          {
            B(r, c) *= scale;
          }
        }
      }

      return scaled;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::setLayout()
    {
      series_z_first_  = N;
      shunt_y1_first_  = series_z_first_ + static_cast<std::size_t>(series_z_.size());
      shunt_y2_first_  = shunt_y1_first_ + static_cast<std::size_t>(shunt_y1_.size());
      series_z_output_ = shunt_y1_first_ - N;
      shunt_y1_output_ = shunt_y2_first_ - N;
      shunt_y2_output_ = shunt_y2_first_ + static_cast<std::size_t>(shunt_y2_.size()) - N;
      size_            = static_cast<IdxT>(shunt_y2_first_ + static_cast<std::size_t>(shunt_y2_.size()));
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::setGridKitComponentID(IdxT gridkit_component_id)
    {
      gridkit_component_id_ = gridkit_component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::verify() const
    {
      int ret = 0;

      if (bus1_ == nullptr)
      {
        ::GridKit::Utilities::Logger::error() << "LineLumped: bus1 pointer is null\n";
        ++ret;
      }

      if (bus2_ == nullptr)
      {
        ::GridKit::Utilities::Logger::error() << "LineLumped: bus2 pointer is null\n";
        ++ret;
      }

      if (bus1_ != nullptr && bus1_->phaseCount() != static_cast<IdxT>(N))
      {
        ::GridKit::Utilities::Logger::error() << "LineLumped: bus1 phase count does not match model dimension\n";
        ++ret;
      }

      if (bus2_ != nullptr && bus2_->phaseCount() != static_cast<IdxT>(N))
      {
        ::GridKit::Utilities::Logger::error() << "LineLumped: bus2 phase count does not match model dimension\n";
        ++ret;
      }

      if (!std::isfinite(data_.dx) || data_.dx <= RealT{0.0})
      {
        ::GridKit::Utilities::Logger::error() << "LineLumped: dx must be finite and positive\n";
        ++ret;
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        if (!std::isfinite(data_.i0[n]))
        {
          ::GridKit::Utilities::Logger::error() << "LineLumped: i0 contains a non-finite value\n";
          ++ret;
        }
      }

      ret += series_z_.verify();
      ret += shunt_y1_.verify();
      ret += shunt_y2_.verify();

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::wireSubmodelSignals()
    {
      for (std::size_t n = 0; n < N; ++n)
      {
        const auto idx = static_cast<IdxT>(n);

        i_nodes_[n].set(&y_[n], &yp_[n], &variable_indices_[n], true);
        v1_nodes_[n].set(&bus1_->V(idx), &bus1_->Vp(idx), &bus1_->getVariableIndex(idx), true);
        v2_nodes_[n].set(&bus2_->V(idx), &bus2_->Vp(idx), &bus2_->getVariableIndex(idx), true);

        series_z_.getSignals().template attachSignalNode<VectorFitExternalVariables::INPUT>(n, 0, &i_nodes_[n]);
        shunt_y1_.getSignals().template attachSignalNode<VectorFitExternalVariables::INPUT>(n, 0, &v1_nodes_[n]);
        shunt_y2_.getSignals().template attachSignalNode<VectorFitExternalVariables::INPUT>(n, 0, &v2_nodes_[n]);

        series_z_.getSignals().template assignSignalNode<VectorFitInternalVariables::Y_OUT>(n, 0, &z_out_nodes_[n]);
        shunt_y1_.getSignals().template assignSignalNode<VectorFitInternalVariables::Y_OUT>(n, 0, &y1_out_nodes_[n]);
        shunt_y2_.getSignals().template assignSignalNode<VectorFitInternalVariables::Y_OUT>(n, 0, &y2_out_nodes_[n]);
      }
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::bindSubmodels()
    {
      series_z_.bind(y_, yp_, f_, tag_, abs_tol_, static_cast<IdxT>(series_z_first_));
      shunt_y1_.bind(y_, yp_, f_, tag_, abs_tol_, static_cast<IdxT>(shunt_y1_first_));
      shunt_y2_.bind(y_, yp_, f_, tag_, abs_tol_, static_cast<IdxT>(shunt_y2_first_));
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::copyParentIndicesToSubmodels()
    {
      for (IdxT j = 0; j < series_z_.size(); ++j)
      {
        const auto parent = series_z_first_ + static_cast<std::size_t>(j);
        series_z_.setVariableIndex(j, variable_indices_[parent]);
        series_z_.setResidualIndex(j, residual_indices_[parent]);
      }

      for (IdxT j = 0; j < shunt_y1_.size(); ++j)
      {
        const auto parent = shunt_y1_first_ + static_cast<std::size_t>(j);
        shunt_y1_.setVariableIndex(j, variable_indices_[parent]);
        shunt_y1_.setResidualIndex(j, residual_indices_[parent]);
      }

      for (IdxT j = 0; j < shunt_y2_.size(); ++j)
      {
        const auto parent = shunt_y2_first_ + static_cast<std::size_t>(j);
        shunt_y2_.setVariableIndex(j, variable_indices_[parent]);
        shunt_y2_.setResidualIndex(j, residual_indices_[parent]);
      }
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::allocate()
    {
      if (bus1_ == nullptr || bus2_ == nullptr
          || bus1_->phaseCount() != static_cast<IdxT>(N)
          || bus2_->phaseCount() != static_cast<IdxT>(N))
      {
        return 1;
      }

      const auto size = static_cast<std::size_t>(size_);

      if (!allocated_)
      {
        allocateVectors(size_);
      }

      assert(y_.size() == size);
      assert(yp_.size() == size);
      assert(f_.size() == size);
      assert(tag_.size() == size);
      assert(abs_tol_.size() == size);

      variable_indices_.resize(size);
      residual_indices_.resize(size);

      for (IdxT j = 0; j < size_; ++j)
      {
        variable_indices_[static_cast<std::size_t>(j)] = offset_ + j;
        residual_indices_[static_cast<std::size_t>(j)] = offset_ + j;
      }

      wireSubmodelSignals();
      bindSubmodels();

      int ret  = 0;
      ret     += series_z_.allocate();
      ret     += shunt_y1_.allocate();
      ret     += shunt_y2_.allocate();
      copyParentIndicesToSubmodels();

      nnz_ = 0;
      J_.zeroMatrix();
      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::initialize()
    {
      for (std::size_t n = 0; n < N; ++n)
      {
        y_[n]  = data_.i0[n];
        yp_[n] = ScalarT{0.0};
      }

      int ret  = 0;
      ret     += series_z_.initialize();
      ret     += shunt_y1_.initialize();
      ret     += shunt_y2_.initialize();

      evaluateTerminalCurrents();
      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::tagDifferentiable()
    {
      for (std::size_t j = 0; j < tag_.size(); ++j)
      {
        tag_[j] = ScalarT{0.0};
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        tag_[n] = ScalarT{1.0};
      }

      int ret  = 0;
      ret     += series_z_.tagDifferentiable();
      ret     += shunt_y1_.tagDifferentiable();
      ret     += shunt_y2_.tagDifferentiable();

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::setAbsoluteTolerance(RealT rel_tol)
    {
      for (std::size_t j = 0; j < abs_tol_.size(); ++j)
      {
        abs_tol_[j] = rel_tol;
      }

      int ret  = 0;
      ret     += series_z_.setAbsoluteTolerance(rel_tol);
      ret     += shunt_y1_.setAbsoluteTolerance(rel_tol);
      ret     += shunt_y2_.setAbsoluteTolerance(rel_tol);

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::evaluateSubmodelResiduals()
    {
      int ret  = 0;
      ret     += series_z_.evaluateResidual();
      ret     += shunt_y1_.evaluateResidual();
      ret     += shunt_y2_.evaluateResidual();
      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::evaluateTerminalCurrents()
    {
      for (std::size_t n = 0; n < N; ++n)
      {
        f_[n]      = seriesVoltageDrop(n) + v2(n) - v1(n);
        i_inj1_[n] = -shuntCurrent1(n) - i(n);
        i_inj2_[n] = -shuntCurrent2(n) + i(n);
      }
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::evaluateResidual()
    {
      const int ret = evaluateSubmodelResiduals();
      evaluateTerminalCurrents();

      this->addCurrentInjection(*bus1_, i_inj1_.data());
      this->addCurrentInjection(*bus2_, i_inj2_.data());

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N>
    void LineLumped<scalar_type, index_type, N>::updateTime(RealT t, RealT a)
    {
      BaseT::updateTime(t, a);
      series_z_.updateTime(t, a);
      shunt_y1_.updateTime(t, a);
      shunt_y2_.updateTime(t, a);
    }

#ifndef GRIDKIT_ENABLE_ENZYME
    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::evaluateJacobian()
    {
      return 0;
    }
#endif
  } // namespace EMT
} // namespace GridKit
