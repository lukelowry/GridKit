/**
 * @file Delay.cpp
 *
 * @brief Initialization, equation residual, and sparse-jet Jacobian for the Delay model.
 *
 */

#include "Delay.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <GridKit/LinearAlgebra/DenseExpression.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename ScalarT, typename IdxT>
        Delay<ScalarT, IdxT>::Delay(DelayData<ScalarT, IdxT> data, SignalT input)
          : data_(data),
            input_(input),
            layout_(lagCount(data)),
            n_(static_cast<ScalarT>(layout_.n))
        {
          validateInput(input_);
        }

        template <typename ScalarT, typename IdxT>
        IdxT Delay<ScalarT, IdxT>::lagCount(const DelayData<ScalarT, IdxT>& data)
        {
          if (!std::isfinite(static_cast<double>(data.delay)) || data.delay <= ScalarT{0.0})
          {
            throw std::runtime_error("Delay requires a positive finite delay");
          }
          if (!std::isfinite(static_cast<double>(data.dt_min)) || data.dt_min <= ScalarT{0.0})
          {
            throw std::runtime_error("Delay requires a positive finite dt_min");
          }

          const ScalarT raw = std::floor(data.delay / data.dt_min);
          if (!std::isfinite(static_cast<double>(raw))
              || raw > static_cast<ScalarT>(std::numeric_limits<IdxT>::max()))
          {
            throw std::runtime_error("Delay lag count is not representable");
          }

          return static_cast<IdxT>(std::max(ScalarT{1.0}, raw));
        }

        template <typename ScalarT, typename IdxT>
        void Delay<ScalarT, IdxT>::validateInput(SignalT input)
        {
          if (input.rows != 1 || input.cols != 1)
          {
            throw std::runtime_error("Delay input signal must be scalar");
          }
          if (input.storage != SignalStorageT::State)
          {
            throw std::runtime_error("Delay input signal must use state storage");
          }
        }

        template <typename ScalarT, typename IdxT>
        int Delay<ScalarT, IdxT>::initialize()
        {
          auto x       = state(layout_.x, layout_.n);
          auto xd      = derivative(layout_.x, layout_.n);
          auto input_u = input(input_);

          for (IdxT i = 0; i < layout_.n; ++i)
          {
            x[i]  = input_u[0];
            xd[i] = ScalarT{0.0};
          }

          return 0;
        }

        template <typename ScalarT, typename IdxT>
        template <typename V>
        __attribute__((always_inline)) inline void
        Delay<ScalarT, IdxT>::evaluateResidualKernel(V* y, V* yp, V* u, V* f) const
        {
          using namespace GridKit::LinearAlgebra;

          const Layout& L       = layout_;
          auto          x       = slice(y, L.x, L.n);
          auto          xd      = slice(yp, L.x, L.n);
          auto          Fx      = slice(f, L.x, L.n);
          auto          input_u = input(u, input_);
          auto          A       = matrix<V>(L.n, L.n, [](IdxT i, IdxT j) -> V
                             { return i == j ? V{-1.0} : (i == j + 1 ? V{1.0} : V{0.0}); });
          auto          b       = vector<V>(L.n, [](IdxT i) -> V
                             { return i == 0 ? V{1.0} : V{0.0}; });

          equation(Fx) = -data_.delay * xd + n_ * (A * x + b * input_u[0]);
        }

        template <typename ScalarT, typename IdxT>
        void Delay<ScalarT, IdxT>::evaluateInternalResidual(ScalarT* y,
                                                            ScalarT* yp,
                                                            ScalarT* u,
                                                            ScalarT* f) const
        {
          evaluateResidualKernel<ScalarT>(y, yp, u, f);
        }

        template <typename ScalarT, typename IdxT>
        void Delay<ScalarT, IdxT>::tagDifferentiable(std::vector<bool>& tag) const
        {
          for (IdxT i = 0; i < size(); ++i)
          {
            tag[static_cast<size_t>(base_ + i)] = true;
          }
        }

        template <typename ScalarT, typename IdxT>
        int Delay<ScalarT, IdxT>::evaluateResidual()
        {
          evaluateInternalResidual(y_, yp_, nullptr, f_);
          return 0;
        }

        template <typename ScalarT, typename IdxT>
        int Delay<ScalarT, IdxT>::evaluateJacobian()
        {
          return this->template evaluateInputDifferentialJacobian<Delay<ScalarT, IdxT>>();
        }

        template class Delay<double, long int>;
        template class Delay<double, size_t>;
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
