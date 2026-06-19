/**
 * @file StateSpace.cpp
 *
 * @brief Initialization, equation residual, and sparse-jet Jacobian for StateSpace.
 *
 */

#include "StateSpace.hpp"

#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <utility>

#include <GridKit/LinearAlgebra/DenseExpression.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Rational
      {
        namespace
        {
          template <typename ScalarT>
          bool finite(ScalarT value)
          {
            return std::isfinite(static_cast<double>(value));
          }

          template <typename ScalarT>
          bool finite(std::complex<ScalarT> value)
          {
            return finite(value.real()) && finite(value.imag());
          }

          template <typename ScalarT>
          ScalarT tolerance(ScalarT scale = ScalarT{1.0})
          {
            return static_cast<ScalarT>(100.0) * std::numeric_limits<ScalarT>::epsilon() * scale;
          }

          template <typename ScalarT>
          bool close(std::complex<ScalarT> a, std::complex<ScalarT> b)
          {
            const ScalarT scale = ScalarT{1.0} + std::abs(a) + std::abs(b);
            return std::abs(a - b) <= tolerance(scale);
          }

          template <typename ScalarT>
          bool realValued(std::complex<ScalarT> value)
          {
            const ScalarT scale = ScalarT{1.0} + std::abs(value.real());
            return std::abs(value.imag()) <= tolerance(scale);
          }
        } // namespace

        template <typename ScalarT, typename IdxT>
        StateSpace<ScalarT, IdxT>::StateSpace(StateSpaceData<ScalarT, IdxT> data,
                                              SignalT                       input)
          : data_(std::move(data)),
            input_(input),
            input_dot_(input),
            layout_(data_.N, data_.K, data_.Q)
        {
          input_dot_.storage = SignalStorageT::Derivative;
          validateData();
          validateInput(input_, layout_.K);
          deriveRealParameters();
        }

        template <typename ScalarT, typename IdxT>
        void StateSpace<ScalarT, IdxT>::validateInput(SignalT input, IdxT K)
        {
          if (input.rows != K || input.cols != 1)
          {
            throw std::runtime_error("StateSpace input signal must be a K-vector");
          }
          if (input.storage != SignalStorageT::State)
          {
            throw std::runtime_error("StateSpace input signal must use state storage");
          }
        }

        template <typename ScalarT, typename IdxT>
        void StateSpace<ScalarT, IdxT>::validateData() const
        {
          const Layout& L = layout_;
          if (L.N == 0 || L.K == 0 || L.Q == 0)
          {
            throw std::runtime_error("StateSpace requires positive N, K, and Q");
          }

          const size_t NK = static_cast<size_t>(L.N * L.K);
          const size_t NQ = static_cast<size_t>(L.N * L.Q);
          const size_t QK = static_cast<size_t>(L.Q * L.K);
          if (data_.D.size() != NK || data_.E.size() != NK)
          {
            throw std::runtime_error("StateSpace D and E sizes must be N x K");
          }
          if (data_.poles.size() != static_cast<size_t>(L.Q))
          {
            throw std::runtime_error("StateSpace pole count must match Q");
          }
          if (data_.C.size() != NQ)
          {
            throw std::runtime_error("StateSpace C size must be N x Q");
          }
          if (data_.B.size() != QK)
          {
            throw std::runtime_error("StateSpace B size must be Q x K");
          }

          for (const auto value : data_.D)
          {
            if (!finite(value))
            {
              throw std::runtime_error("StateSpace D entries must be finite");
            }
          }
          for (const auto value : data_.E)
          {
            if (!finite(value))
            {
              throw std::runtime_error("StateSpace E entries must be finite");
            }
          }
          for (const auto value : data_.poles)
          {
            if (!finite(value))
            {
              throw std::runtime_error("StateSpace pole entries must be finite");
            }
            if (value == typename StateSpaceData<ScalarT, IdxT>::ComplexT{0.0, 0.0})
            {
              throw std::runtime_error("StateSpace poles must be nonzero");
            }
          }
          for (const auto value : data_.C)
          {
            if (!finite(value))
            {
              throw std::runtime_error("StateSpace C entries must be finite");
            }
          }
          for (const auto value : data_.B)
          {
            if (!finite(value))
            {
              throw std::runtime_error("StateSpace B entries must be finite");
            }
          }

          for (IdxT q = 0; q < L.Q;)
          {
            const auto pole = data_.poles[static_cast<size_t>(q)];
            if (realValued(pole))
            {
              for (IdxT i = 0; i < L.N; ++i)
              {
                if (!realValued(data_.C[static_cast<size_t>(i * L.Q + q)]))
                {
                  throw std::runtime_error("StateSpace real poles require real C columns");
                }
              }
              for (IdxT k = 0; k < L.K; ++k)
              {
                if (!realValued(data_.B[static_cast<size_t>(q * L.K + k)]))
                {
                  throw std::runtime_error("StateSpace real poles require real B rows");
                }
              }
              ++q;
              continue;
            }

            if (q + 1 >= L.Q)
            {
              throw std::runtime_error("StateSpace complex poles must have adjacent conjugate pairs");
            }

            const auto paired = data_.poles[static_cast<size_t>(q + 1)];
            if (!close(pole, std::conj(paired)))
            {
              throw std::runtime_error("StateSpace complex poles must be adjacent conjugate pairs");
            }
            for (IdxT i = 0; i < L.N; ++i)
            {
              const auto c0 = data_.C[static_cast<size_t>(i * L.Q + q)];
              const auto c1 = data_.C[static_cast<size_t>(i * L.Q + q + 1)];
              if (!close(c0, std::conj(c1)))
              {
                throw std::runtime_error("StateSpace C columns must follow pole conjugate ordering");
              }
            }
            for (IdxT k = 0; k < L.K; ++k)
            {
              const auto b0 = data_.B[static_cast<size_t>(q * L.K + k)];
              const auto b1 = data_.B[static_cast<size_t>((q + 1) * L.K + k)];
              if (!close(b0, std::conj(b1)))
              {
                throw std::runtime_error("StateSpace B rows must follow pole conjugate ordering");
              }
            }
            q += 2;
          }
        }

        template <typename ScalarT, typename IdxT>
        void StateSpace<ScalarT, IdxT>::deriveRealParameters()
        {
          const Layout& L = layout_;

          a_.assign(static_cast<size_t>(L.Q), ScalarT{0.0});
          omega_.assign(static_cast<size_t>(L.Q), ScalarT{0.0});
          Cr_.assign(static_cast<size_t>(L.N * L.Q), ScalarT{0.0});
          Ci_.assign(static_cast<size_t>(L.N * L.Q), ScalarT{0.0});
          Br_.assign(static_cast<size_t>(L.Q * L.K), ScalarT{0.0});
          Bi_.assign(static_cast<size_t>(L.Q * L.K), ScalarT{0.0});

          for (IdxT q = 0; q < L.Q; ++q)
          {
            const auto pole                = data_.poles[static_cast<size_t>(q)];
            a_[static_cast<size_t>(q)]     = pole.real();
            omega_[static_cast<size_t>(q)] = realValued(pole) ? ScalarT{0.0} : pole.imag();
          }
          for (IdxT i = 0; i < L.N; ++i)
          {
            for (IdxT q = 0; q < L.Q; ++q)
            {
              const auto idx = static_cast<size_t>(i * L.Q + q);
              Cr_[idx]       = data_.C[idx].real();
              Ci_[idx]       = realValued(data_.C[idx]) ? ScalarT{0.0} : data_.C[idx].imag();
            }
          }
          for (IdxT q = 0; q < L.Q; ++q)
          {
            for (IdxT k = 0; k < L.K; ++k)
            {
              const auto idx = static_cast<size_t>(q * L.K + k);
              Br_[idx]       = data_.B[idx].real();
              Bi_[idx]       = realValued(data_.B[idx]) ? ScalarT{0.0} : data_.B[idx].imag();
            }
          }
        }

        template <typename ScalarT, typename IdxT>
        int StateSpace<ScalarT, IdxT>::initialize()
        {
          using ComplexT = typename StateSpaceData<ScalarT, IdxT>::ComplexT;

          const Layout& L   = layout_;
          auto          xr  = state(L.xr, L.Q);
          auto          xi  = state(L.xi, L.Q);
          auto          out = state(L.out, L.N);
          auto          xrd = derivative(L.xr, L.Q);
          auto          xid = derivative(L.xi, L.Q);
          auto          yd  = derivative(L.out, L.N);
          auto          in  = input(input_);
          auto          ind = input(input_dot_);

          for (IdxT q = 0; q < L.Q; ++q)
          {
            ComplexT Bu{0.0, 0.0};
            ComplexT Bud{0.0, 0.0};
            for (IdxT k = 0; k < L.K; ++k)
            {
              const auto Bqk  = data_.B[static_cast<size_t>(q * L.K + k)];
              Bu             += Bqk * in[k];
              Bud            += Bqk * ind[k];
            }

            const ComplexT p{a_[static_cast<size_t>(q)], omega_[static_cast<size_t>(q)]};
            const auto     x0 = -Bu / p - Bud / (p * p);

            xr[q]  = x0.real();
            xi[q]  = x0.imag();
            xrd[q] = a_[static_cast<size_t>(q)] * xr[q]
                     - omega_[static_cast<size_t>(q)] * xi[q] + Bu.real();
            xid[q] = omega_[static_cast<size_t>(q)] * xr[q]
                     + a_[static_cast<size_t>(q)] * xi[q] + Bu.imag();
          }

          for (IdxT i = 0; i < L.N; ++i)
          {
            out[i] = ScalarT{0.0};
            yd[i]  = ScalarT{0.0};
            for (IdxT k = 0; k < L.K; ++k)
            {
              out[i] += d(i, k) * in[k] + e(i, k) * ind[k];
              yd[i]  += d(i, k) * ind[k];
            }
            for (IdxT q = 0; q < L.Q; ++q)
            {
              out[i] += cr(i, q) * xr[q] - ci(i, q) * xi[q];
              yd[i]  += cr(i, q) * xrd[q] - ci(i, q) * xid[q];
            }
          }

          return 0;
        }

        template <typename ScalarT, typename IdxT>
        template <typename V>
        __attribute__((always_inline)) inline void
        StateSpace<ScalarT, IdxT>::evaluateResidualKernel(V* y, V* yp, V* u, V* f) const
        {
          using namespace GridKit::LinearAlgebra;

          const Layout& L    = layout_;
          auto          xr   = slice(y, L.xr, L.Q);
          auto          xi   = slice(y, L.xi, L.Q);
          auto          out  = slice(y, L.out, L.N);
          auto          xrd  = slice(yp, L.xr, L.Q);
          auto          xid  = slice(yp, L.xi, L.Q);
          auto          Fxr  = slice(f, L.xr, L.Q);
          auto          Fxi  = slice(f, L.xi, L.Q);
          auto          Fout = slice(f, L.out, L.N);
          auto          in   = input(u, input_);
          auto          ind  = input(u, input_dot_);

          auto A     = diag(vector<V>(L.Q, [&](IdxT q) -> V
                                  { return V{a_[static_cast<size_t>(q)]}; }));
          auto Omega = diag(vector<V>(L.Q, [&](IdxT q) -> V
                                      { return V{omega_[static_cast<size_t>(q)]}; }));
          auto D     = matrix<V>(L.N, L.K, [&](IdxT i, IdxT k) -> V
                             { return V{d(i, k)}; });
          auto E     = matrix<V>(L.N, L.K, [&](IdxT i, IdxT k) -> V
                             { return V{e(i, k)}; });
          auto Cr    = matrix<V>(L.N, L.Q, [&](IdxT i, IdxT q) -> V
                              { return V{cr(i, q)}; });
          auto Ci    = matrix<V>(L.N, L.Q, [&](IdxT i, IdxT q) -> V
                              { return V{ci(i, q)}; });
          auto Br    = matrix<V>(L.Q, L.K, [&](IdxT q, IdxT k) -> V
                              { return V{br(q, k)}; });
          auto Bi    = matrix<V>(L.Q, L.K, [&](IdxT q, IdxT k) -> V
                              { return V{bi(q, k)}; });

          equation(Fxr)  = -xrd + A * xr - Omega * xi + Br * in;
          equation(Fxi)  = -xid + Omega * xr + A * xi + Bi * in;
          equation(Fout) = -out + D * in + E * ind + Cr * xr - Ci * xi;
        }

        template <typename ScalarT, typename IdxT>
        void StateSpace<ScalarT, IdxT>::evaluateInternalResidual(ScalarT* y,
                                                                 ScalarT* yp,
                                                                 ScalarT* u,
                                                                 ScalarT* f) const
        {
          evaluateResidualKernel<ScalarT>(y, yp, u, f);
        }

        template <typename ScalarT, typename IdxT>
        void StateSpace<ScalarT, IdxT>::tagDifferentiable(std::vector<bool>& tag) const
        {
          for (IdxT i = 0; i < layout_.out; ++i)
          {
            tag[static_cast<size_t>(base_ + i)] = true;
          }
        }

        template <typename ScalarT, typename IdxT>
        int StateSpace<ScalarT, IdxT>::evaluateResidual()
        {
          evaluateInternalResidual(y_, yp_, nullptr, f_);
          return 0;
        }

        template <typename ScalarT, typename IdxT>
        int StateSpace<ScalarT, IdxT>::evaluateJacobian()
        {
          return this->template evaluateInputDifferentialJacobian<StateSpace<ScalarT, IdxT>>();
        }

        template class StateSpace<double, long int>;
        template class StateSpace<double, size_t>;
      } // namespace Rational
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
