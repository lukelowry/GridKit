#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include <GridKit/Constants.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFit.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    VectorFit<scalar_type, index_type, N, K>::VectorFit()
    {
      size_ = static_cast<IdxT>(N);
      signals_.template setExternalPortCount<VectorFitExternalVariables::INPUT>(K);
      signals_.template setInternalPortCount<VectorFitInternalVariables::Y_OUT>(N);
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    VectorFit<scalar_type, index_type, N, K>::VectorFit(const ModelDataT& data)
      : VectorFit()
    {
      setData(data);
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    void VectorFit<scalar_type, index_type, N, K>::setData(const ModelDataT& data)
    {
      D_          = data.D;
      E_          = data.E;
      poles_      = data.poles;
      A_          = data.A;
      B_          = data.B;
      pole_count_ = poles_.size();
      size_       = static_cast<IdxT>(yOutOffset() + N);
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    std::size_t VectorFit<scalar_type, index_type, N, K>::yOutOffset() const
    {
      return 2 * pole_count_ * K;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::setGridKitComponentID(IdxT gridkit_component_id)
    {
      gridkit_component_id_ = gridkit_component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::verify() const
    {
      int ret = 0;

      for (std::size_t k = 0; k < K; ++k)
      {
        if (!signals_.template isAttached<VectorFitExternalVariables::INPUT>(k, 0))
        {
          Log::error() << "VectorFit: input signal " << k << " is not attached\n";
          ++ret;
          continue;
        }

        if (!signals_.template isLinked<VectorFitExternalVariables::INPUT>(k, 0))
        {
          Log::error() << "VectorFit: input signal " << k << " is attached with no linked source\n";
          ++ret;
        }
        else if (!signals_.template isExternalVariableDifferential<VectorFitExternalVariables::INPUT>(k, 0))
        {
          Log::error() << "VectorFit: input signal " << k << " is not differential\n";
          ++ret;
        }
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        if (!signals_.template isAssigned<VectorFitInternalVariables::Y_OUT>(n, 0))
        {
          Log::error() << "VectorFit: y_out signal " << n << " is not assigned\n";
          ++ret;
        }
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        for (std::size_t k = 0; k < K; ++k)
        {
          if (!std::isfinite(D_(n, k)) || !std::isfinite(E_(n, k)))
          {
            Log::error() << "VectorFit: non-finite D or E parameter\n";
            ++ret;
          }
        }
      }

      const bool valid_residue_count =
          poles_.size() == pole_count_ && A_.size() == pole_count_ && B_.size() == pole_count_;
      if (!valid_residue_count)
      {
        Log::error() << "VectorFit: pole and residue counts do not match\n";
        ++ret;
      }

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        const RealT a     = poles_[q][0];
        const RealT omega = poles_[q][1];
        const RealT den   = a * a + omega * omega;
        if (!std::isfinite(a) || !std::isfinite(omega) || den == RealT{0.0})
        {
          Log::error() << "VectorFit: invalid pole block " << q << "\n";
          ++ret;
        }

        if (!valid_residue_count)
        {
          continue;
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          for (std::size_t k = 0; k < K; ++k)
          {
            if (!std::isfinite(A_[q](n, k)) || !std::isfinite(B_[q](n, k)))
            {
              Log::error() << "VectorFit: non-finite residue parameter\n";
              ++ret;
            }
          }
        }
      }

      return ret;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::allocate()
    {
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

      wb_.assign(1, ScalarT{0.0});
      ws_.assign(K, ScalarT{0.0});
      wsp_.assign(K, ScalarT{0.0});
      ws_indices_.assign(K, INVALID_INDEX<IdxT>);

      for (IdxT j = 0; j < size_; ++j)
      {
        variable_indices_[static_cast<std::size_t>(j)] = offset_ + j;
        residual_indices_[static_cast<std::size_t>(j)] = offset_ + j;
      }

      const auto y_out = yOutOffset();
      for (std::size_t n = 0; n < N; ++n)
      {
        if (signals_.template isAssigned<VectorFitInternalVariables::Y_OUT>(n, 0))
        {
          signals_.template getSignalNode<VectorFitInternalVariables::Y_OUT>(n, 0)->set(
              &y_[y_out + n],
              &variable_indices_[y_out + n]);
        }
      }

      nnz_ = 0;
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    void VectorFit<scalar_type, index_type, N, K>::readInput()
    {
      for (std::size_t k = 0; k < K; ++k)
      {
        ws_[k]         = signals_.template readExternalVariable<VectorFitExternalVariables::INPUT>(k, 0);
        wsp_[k]        = signals_.template readExternalVariableDerivative<VectorFitExternalVariables::INPUT>(k, 0);
        ws_indices_[k] = signals_.template readExternalVariableIndex<VectorFitExternalVariables::INPUT>(k, 0);
      }
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::initialize()
    {
      readInput();

      const auto* poles = poles_.data();

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        const RealT a     = poles[q][0];
        const RealT omega = poles[q][1];
        const RealT den   = a * a + omega * omega;
        if (den == RealT{0.0})
        {
          return 1;
        }

        const RealT den2 = den * den;

        auto w  = PhasorDynamics::Equation::block<K>(y_.data(), 2 * q);
        auto v  = PhasorDynamics::Equation::block<K>(y_.data(), 2 * q + 1);
        auto wp = PhasorDynamics::Equation::block<K>(yp_.data(), 2 * q);
        auto vp = PhasorDynamics::Equation::block<K>(yp_.data(), 2 * q + 1);

        for (std::size_t k = 0; k < K; ++k)
        {
          w[k] = -a / den * ws_[k]
                 - (a * a - omega * omega) / den2 * wsp_[k];
          v[k] = omega / den * ws_[k]
                 + 2.0 * a * omega / den2 * wsp_[k];

          wp[k] = a * w[k] - omega * v[k] + ws_[k];
          vp[k] = omega * w[k] + a * v[k];
        }
      }

      const auto y_out = yOutOffset();
      for (std::size_t n = 0; n < N; ++n)
      {
        y_[y_out + n]  = ScalarT{0.0};
        yp_[y_out + n] = ScalarT{0.0};
      }

      evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), wsp_.data(), f_.data());
      for (std::size_t n = 0; n < N; ++n)
      {
        y_[y_out + n] -= f_[y_out + n];
      }
      evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), wsp_.data(), f_.data());

      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::tagDifferentiable()
    {
      const auto y_out = yOutOffset();
      for (std::size_t j = 0; j < y_out; ++j)
      {
        tag_[j] = true;
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        tag_[y_out + n] = false;
      }

      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::setAbsoluteTolerance(RealT rel_tol)
    {
      for (std::size_t j = 0; j < abs_tol_.size(); ++j)
      {
        abs_tol_[j] = rel_tol;
      }
      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    __attribute__((always_inline)) int VectorFit<scalar_type, index_type, N, K>::evaluateInternalResidual(
        ScalarT* y, ScalarT* yp, [[maybe_unused]] ScalarT* wb, ScalarT* ws, ScalarT* wsp, ScalarT* f)
    {
      namespace Eq = GridKit::PhasorDynamics::Equation;

      auto        input     = Eq::block<K>(ws, 0);
      auto        input_dot = Eq::block<K>(wsp, 0);
      const auto* poles     = poles_.data();
      const auto* A         = A_.data();
      const auto* B         = B_.data();

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        const RealT a     = poles[q][0];
        const RealT omega = poles[q][1];

        auto w  = Eq::block<K>(y, 2 * q);
        auto v  = Eq::block<K>(y, 2 * q + 1);
        auto wp = Eq::block<K>(yp, 2 * q);
        auto vp = Eq::block<K>(yp, 2 * q + 1);

        auto fw = Eq::block<K>(f, 2 * q);
        auto fv = Eq::block<K>(f, 2 * q + 1);

        for (std::size_t k = 0; k < K; ++k)
        {
          fw[k] = -wp[k] + a * w[k] - omega * v[k] + input[k];
          fv[k] = -vp[k] + omega * w[k] + a * v[k];
        }
      }

      const auto y_out_offset = yOutOffset();
      auto       y_out        = Eq::slice<N>(y, y_out_offset);
      auto       f_out        = Eq::slice<N>(f, y_out_offset);

      Eq::Vec<ScalarT, N> residual_out = y_out - D_ * input - E_ * input_dot;

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        auto w = Eq::block<K>(y, 2 * q);
        auto v = Eq::block<K>(y, 2 * q + 1);

        residual_out = residual_out - A[q] * w + B[q] * v;
      }

      f_out = residual_out;

      return 0;
    }

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::evaluateResidual()
    {
      readInput();
      return evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), wsp_.data(), f_.data());
    }

#ifndef GRIDKIT_ENABLE_ENZYME
    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::evaluateJacobian()
    {
      return 0;
    }
#endif
  } // namespace EMT
} // namespace GridKit
