#include "Delay.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    Delay<ScalarT, IdxT>::Delay(RealT tau, RealT fmax, InputFunction input, InputFunction input_derivative, InitialStateFunction initial_state)
      : tau_(tau),
        fmax_(fmax),
        n_(computeSectionCount(tau, fmax)),
        T_(tau_ / static_cast<RealT>(n_)),
        input_function_(std::move(input)),
        input_derivative_(std::move(input_derivative)),
        initial_state_function_(std::move(initial_state))
    {
      if (!input_function_ || !input_derivative_)
        throw std::invalid_argument("Delay requires input and input derivative functions.");

      initializeStorage();
      initializeJacobian();
      updateTime(0.0, 0.0);
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::computeSectionCount(RealT tau, RealT fmax)
    {
      if (tau <= RealT{0})
        throw std::invalid_argument("Delay tau must be positive.");
      if (fmax <= RealT{0})
        throw std::invalid_argument("Delay fmax must be positive.");
      return std::max<IdxT>(1, static_cast<IdxT>(std::ceil(fmax * tau)));
    }

    template <class ScalarT, typename IdxT>
    void Delay<ScalarT, IdxT>::initializeStorage()
    {
      y_.assign(static_cast<size_t>(n_), ScalarT{0});
      yp_.assign(static_cast<size_t>(n_), ScalarT{0});
      tag_.assign(static_cast<size_t>(n_), true);
      abs_tol_.assign(static_cast<size_t>(n_), ScalarT{0});
      f_.assign(static_cast<size_t>(n_), ScalarT{0});
    }

    template <class ScalarT, typename IdxT>
    void Delay<ScalarT, IdxT>::initializeJacobian()
    {
      const IdxT nnz_value = nnz();
      auto*      rows      = new IdxT[static_cast<size_t>(n_) + 1];
      auto*      cols      = new IdxT[static_cast<size_t>(nnz_value)];
      auto*      vals      = new RealT[static_cast<size_t>(nnz_value)];

      rows[0]     = 0;
      IdxT cursor = 0;
      for (IdxT row = 0; row < n_; ++row)
      {
        if (row > 0)
          cols[cursor++] = row - 1;
        cols[cursor++] = row;
        rows[row + 1]  = cursor;
      }

      std::fill(vals, vals + nnz_value, RealT{0});
      csr_jac_ = std::make_unique<CsrMatrixT>(n_, n_, nnz_value, &rows, &cols, &vals);
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::allocate()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::initialize()
    {
      for (IdxT i = 0; i < n_; ++i)
      {
        auto& y  = y_[static_cast<size_t>(i)];
        auto& yp = yp_[static_cast<size_t>(i)];
        if (initial_state_function_)
        {
          initial_state_function_(i, time_, T_, y, yp);
        }
        else
        {
          const RealT history_time = time_ - static_cast<RealT>(i + 1) * T_;
          y                        = input_function_(history_time);
          yp                       = input_derivative_(history_time);
        }
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::tagDifferentiable()
    {
      std::fill(tag_.begin(), tag_.end(), true);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::setAbsoluteTolerance(RealT rel_tol)
    {
      std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::evaluateResidual()
    {
      f_[0] = -T_ * yp_[0] - y_[0] + input_value_;
      for (IdxT i = 1; i < n_; ++i)
        f_[static_cast<size_t>(i)] = -T_ * yp_[static_cast<size_t>(i)] - y_[static_cast<size_t>(i)] + y_[static_cast<size_t>(i - 1)];
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::evaluateJacobian()
    {
      RealT* vals   = csr_jac_->getValues();
      IdxT   cursor = 0;
      for (IdxT row = 0; row < n_; ++row)
      {
        if (row > 0)
          vals[cursor++] = RealT{1};
        vals[cursor++] = -RealT{1} - alpha_ * T_;
      }
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::evaluateIntegrand()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::initializeAdjoint()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::evaluateAdjointResidual()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int Delay<ScalarT, IdxT>::evaluateAdjointIntegrand()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::size()
    {
      return n_;
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::nnz()
    {
      return 2 * n_ - 1;
    }

    template <class ScalarT, typename IdxT>
    bool Delay<ScalarT, IdxT>::hasJacobian()
    {
      return true;
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::sizeQuadrature()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::sizeParams()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    void Delay<ScalarT, IdxT>::updateTime(RealT t, RealT a)
    {
      time_        = t;
      alpha_       = a;
      input_value_ = input_function_(time_);
    }

    template <class ScalarT, typename IdxT>
    typename Delay<ScalarT, IdxT>::CsrMatrixT* Delay<ScalarT, IdxT>::getCsrJacobian() const
    {
      return csr_jac_.get();
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::absoluteTolerance()
    {
      return abs_tol_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::absoluteTolerance() const
    {
      return abs_tol_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::y()
    {
      return y_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::y() const
    {
      return y_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::yp()
    {
      return yp_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::yp() const
    {
      return yp_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<bool>& Delay<ScalarT, IdxT>::tag()
    {
      return tag_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<bool>& Delay<ScalarT, IdxT>::tag() const
    {
      return tag_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::yB()
    {
      return yB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::yB() const
    {
      return yB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::ypB()
    {
      return ypB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::ypB() const
    {
      return ypB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::param()
    {
      return param_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::param() const
    {
      return param_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::param_up()
    {
      return param_up_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::param_up() const
    {
      return param_up_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::param_lo()
    {
      return param_lo_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::param_lo() const
    {
      return param_lo_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::getResidual()
    {
      return f_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::getResidual() const
    {
      return f_;
    }

    template <class ScalarT, typename IdxT>
    typename Delay<ScalarT, IdxT>::MatrixT& Delay<ScalarT, IdxT>::getJacobian()
    {
      return jac_;
    }

    template <class ScalarT, typename IdxT>
    const typename Delay<ScalarT, IdxT>::MatrixT& Delay<ScalarT, IdxT>::getJacobian() const
    {
      return jac_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::getIntegrand()
    {
      return g_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::getIntegrand() const
    {
      return g_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::getAdjointResidual()
    {
      return fB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::getAdjointResidual() const
    {
      return fB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& Delay<ScalarT, IdxT>::getAdjointIntegrand()
    {
      return gB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& Delay<ScalarT, IdxT>::getAdjointIntegrand() const
    {
      return gB_;
    }

    template <class ScalarT, typename IdxT>
    IdxT Delay<ScalarT, IdxT>::sectionCount() const
    {
      return n_;
    }

    template <class ScalarT, typename IdxT>
    typename Delay<ScalarT, IdxT>::RealT Delay<ScalarT, IdxT>::sectionTime() const
    {
      return T_;
    }

    template <class ScalarT, typename IdxT>
    ScalarT Delay<ScalarT, IdxT>::input() const
    {
      return input_value_;
    }

    template <class ScalarT, typename IdxT>
    ScalarT Delay<ScalarT, IdxT>::output() const
    {
      return y_.back();
    }

    template class Delay<double, long int>;
    template class Delay<double, int>;
    template class Delay<double, size_t>;

  } // namespace EMT
} // namespace GridKit
