#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFit.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <typename scalar_type, typename index_type>
    VectorFit<scalar_type, index_type>::VectorFit(const ModelDataT& data)
    {
      initializeParameters(data);

      pole_count_    = static_cast<IdxT>(poles_.size());
      output_offset_ = static_cast<IdxT>(3) * pole_count_;
      size_          = output_offset_ + static_cast<IdxT>(3);

      pole_kinds_.resize(static_cast<size_t>(pole_count_), PoleKind::Real);
      for (IdxT q = 0; q < pole_count_; ++q)
      {
        if (!approximatelyEqual(poles_[static_cast<size_t>(q)].imag(), 0.0))
        {
          pole_kinds_[static_cast<size_t>(q)] = PoleKind::ComplexFirst;
          if (q + 1 < pole_count_)
          {
            pole_kinds_[static_cast<size_t>(q + 1)] = PoleKind::ComplexSecond;
            ++q;
          }
        }
      }
    }

    template <typename scalar_type, typename index_type>
    VectorFit<scalar_type, index_type>::~VectorFit()
    {
    }

    template <typename scalar_type, typename index_type>
    void VectorFit<scalar_type, index_type>::initializeParameters(
        const ModelDataT& data)
    {
      using Parameters = typename ModelDataT::Parameters;

      if (data.parameters.contains(Parameters::D))
      {
        D_     = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameters::D));
        has_D_ = true;
      }
      if (data.parameters.contains(Parameters::E))
      {
        E_     = std::get<ABCMatrix<RealT>>(data.parameters.at(Parameters::E));
        has_E_ = true;
      }
      if (data.parameters.contains(Parameters::poles))
      {
        poles_ = std::get<std::vector<std::complex<RealT>>>(
            data.parameters.at(Parameters::poles));
        has_poles_ = true;
      }
      if (data.parameters.contains(Parameters::residues))
      {
        residues_ = std::get<std::vector<ABCMatrix<std::complex<RealT>>>>(
            data.parameters.at(Parameters::residues));
        has_residues_ = true;
      }
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::approximatelyEqual(
        RealT lhs,
        RealT rhs) const
    {
      const RealT scale = std::max({static_cast<RealT>(1.0),
                                    std::abs(lhs),
                                    std::abs(rhs)});
      return std::abs(lhs - rhs)
             <= static_cast<RealT>(128.0)
                    * std::numeric_limits<RealT>::epsilon() * scale;
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::approximatelyConjugate(
        const std::complex<RealT>& lhs,
        const std::complex<RealT>& rhs) const
    {
      return approximatelyEqual(lhs.real(), rhs.real())
             && approximatelyEqual(lhs.imag(), -rhs.imag());
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::matrixIsFinite(
        const ABCMatrix<RealT>& matrix) const
    {
      return std::isfinite(matrix[0][0])
             && std::isfinite(matrix[0][1])
             && std::isfinite(matrix[0][2])
             && std::isfinite(matrix[1][0])
             && std::isfinite(matrix[1][1])
             && std::isfinite(matrix[1][2])
             && std::isfinite(matrix[2][0])
             && std::isfinite(matrix[2][1])
             && std::isfinite(matrix[2][2]);
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::complexMatrixIsFinite(
        const ABCMatrix<std::complex<RealT>>& matrix) const
    {
      return std::isfinite(matrix[0][0].real()) && std::isfinite(matrix[0][0].imag())
             && std::isfinite(matrix[0][1].real()) && std::isfinite(matrix[0][1].imag())
             && std::isfinite(matrix[0][2].real()) && std::isfinite(matrix[0][2].imag())
             && std::isfinite(matrix[1][0].real()) && std::isfinite(matrix[1][0].imag())
             && std::isfinite(matrix[1][1].real()) && std::isfinite(matrix[1][1].imag())
             && std::isfinite(matrix[1][2].real()) && std::isfinite(matrix[1][2].imag())
             && std::isfinite(matrix[2][0].real()) && std::isfinite(matrix[2][0].imag())
             && std::isfinite(matrix[2][1].real()) && std::isfinite(matrix[2][1].imag())
             && std::isfinite(matrix[2][2].real()) && std::isfinite(matrix[2][2].imag());
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::complexMatrixIsReal(
        const ABCMatrix<std::complex<RealT>>& matrix) const
    {
      return approximatelyEqual(matrix[0][0].imag(), 0.0)
             && approximatelyEqual(matrix[0][1].imag(), 0.0)
             && approximatelyEqual(matrix[0][2].imag(), 0.0)
             && approximatelyEqual(matrix[1][0].imag(), 0.0)
             && approximatelyEqual(matrix[1][1].imag(), 0.0)
             && approximatelyEqual(matrix[1][2].imag(), 0.0)
             && approximatelyEqual(matrix[2][0].imag(), 0.0)
             && approximatelyEqual(matrix[2][1].imag(), 0.0)
             && approximatelyEqual(matrix[2][2].imag(), 0.0);
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::complexMatricesAreConjugate(
        const ABCMatrix<std::complex<RealT>>& lhs,
        const ABCMatrix<std::complex<RealT>>& rhs) const
    {
      return approximatelyConjugate(lhs[0][0], rhs[0][0])
             && approximatelyConjugate(lhs[0][1], rhs[0][1])
             && approximatelyConjugate(lhs[0][2], rhs[0][2])
             && approximatelyConjugate(lhs[1][0], rhs[1][0])
             && approximatelyConjugate(lhs[1][1], rhs[1][1])
             && approximatelyConjugate(lhs[1][2], rhs[1][2])
             && approximatelyConjugate(lhs[2][0], rhs[2][0])
             && approximatelyConjugate(lhs[2][1], rhs[2][1])
             && approximatelyConjugate(lhs[2][2], rhs[2][2]);
    }

    template <typename scalar_type, typename index_type>
    bool VectorFit<scalar_type, index_type>::matrixIsZero(
        const ABCMatrix<RealT>& matrix) const
    {
      return matrix[0][0] == 0.0
             && matrix[0][1] == 0.0
             && matrix[0][2] == 0.0
             && matrix[1][0] == 0.0
             && matrix[1][1] == 0.0
             && matrix[1][2] == 0.0
             && matrix[2][0] == 0.0
             && matrix[2][1] == 0.0
             && matrix[2][2] == 0.0;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::setGridKitComponentID(
        IdxT component_id)
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::allocate()
    {
      if (!allocated_)
      {
        this->allocateVectors(size_);
      }

      tag_.resize(static_cast<size_t>(size_));
      variable_indices_.resize(static_cast<size_t>(size_));
      residual_indices_.resize(static_cast<size_t>(size_));

      for (IdxT j = 0; j < size_; ++j)
      {
        this->setVariableIndex(j, j);
        this->setResidualIndex(j, j);
      }

      ws_.resize(3);
      wsp_.resize(3);
      ws_indices_.resize(3, INVALID_INDEX<IdxT>);

      auto* y  = y_.getData();
      auto* yp = yp_.getData();

      if (signals_.template isAssigned<VectorFitInternalVariables::out_a>())
      {
        signals_.template getSignalNode<VectorFitInternalVariables::out_a>()->set(
            &y[static_cast<size_t>(output_offset_)],
            &yp[static_cast<size_t>(output_offset_)],
            &this->getVariableIndex(output_offset_));
      }
      if (signals_.template isAssigned<VectorFitInternalVariables::out_b>())
      {
        signals_.template getSignalNode<VectorFitInternalVariables::out_b>()->set(
            &y[static_cast<size_t>(output_offset_ + 1)],
            &yp[static_cast<size_t>(output_offset_ + 1)],
            &this->getVariableIndex(output_offset_ + 1));
      }
      if (signals_.template isAssigned<VectorFitInternalVariables::out_c>())
      {
        signals_.template getSignalNode<VectorFitInternalVariables::out_c>()->set(
            &y[static_cast<size_t>(output_offset_ + 2)],
            &yp[static_cast<size_t>(output_offset_ + 2)],
            &this->getVariableIndex(output_offset_ + 2));
      }

      allocated_ = true;
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::verify() const
    {
      int errors = 0;

      if (!has_D_ || !has_E_ || !has_poles_ || !has_residues_)
      {
        Log::error() << "VectorFit: D, E, poles, and residues are required\n";
        ++errors;
      }

      if (!matrixIsFinite(D_) || !matrixIsFinite(E_))
      {
        Log::error() << "VectorFit: D and E must contain finite values\n";
        ++errors;
      }

      if (residues_.size() != poles_.size())
      {
        Log::error() << "VectorFit: one residue matrix is required for each pole\n";
        ++errors;
      }

      const auto check_count = std::min(poles_.size(), residues_.size());
      for (size_t q = 0; q < check_count; ++q)
      {
        if (!std::isfinite(poles_[q].real())
            || !std::isfinite(poles_[q].imag())
            || !complexMatrixIsFinite(residues_[q]))
        {
          Log::error() << "VectorFit: poles and residues must be finite\n";
          ++errors;
        }

        if (approximatelyEqual(poles_[q].imag(), 0.0))
        {
          if (!complexMatrixIsReal(residues_[q]))
          {
            Log::error() << "VectorFit: a real pole must have a real residue\n";
            ++errors;
          }
          continue;
        }

        if (q + 1 >= check_count)
        {
          Log::error() << "VectorFit: a nonreal pole is missing its adjacent conjugate\n";
          ++errors;
          break;
        }

        if (!approximatelyConjugate(poles_[q], poles_[q + 1])
            || !complexMatricesAreConjugate(residues_[q], residues_[q + 1]))
        {
          Log::error() << "VectorFit: nonreal poles and residues must be adjacent conjugates\n";
          ++errors;
        }
        ++q;
      }

      static constexpr auto INPUT_A = VectorFitExternalVariables::input_a;
      static constexpr auto INPUT_B = VectorFitExternalVariables::input_b;
      static constexpr auto INPUT_C = VectorFitExternalVariables::input_c;

      const bool input_a_ready = signals_.template isAttached<INPUT_A>()
                                 && signals_.template isLinked<INPUT_A>();
      const bool input_b_ready = signals_.template isAttached<INPUT_B>()
                                 && signals_.template isLinked<INPUT_B>();
      const bool input_c_ready = signals_.template isAttached<INPUT_C>()
                                 && signals_.template isLinked<INPUT_C>();

      if (!input_a_ready || !input_b_ready || !input_c_ready)
      {
        Log::error() << "VectorFit: all three input signals must be attached and linked\n";
        ++errors;
      }

      if (!signals_.template isAssigned<VectorFitInternalVariables::out_a>()
          || !signals_.template isAssigned<VectorFitInternalVariables::out_b>()
          || !signals_.template isAssigned<VectorFitInternalVariables::out_c>())
      {
        Log::error() << "VectorFit: all three output signals must be assigned\n";
        ++errors;
      }

      const bool derivative_required = pole_count_ > 0 || !matrixIsZero(E_);
      if (derivative_required && input_a_ready && input_b_ready && input_c_ready)
      {
        if (!signals_.template isDerivativeLinked<INPUT_A>()
            || !signals_.template isDerivativeLinked<INPUT_B>()
            || !signals_.template isDerivativeLinked<INPUT_C>())
        {
          Log::error() << "VectorFit: dynamic input requires all three input derivatives\n";
          ++errors;
        }
      }

      return errors;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::initialize()
    {
      std::fill_n(y_.getData(), static_cast<size_t>(size_), ScalarT{0.0});
      std::fill_n(yp_.getData(), static_cast<size_t>(size_), ScalarT{0.0});

      y_.setDataUpdated();
      yp_.setDataUpdated();
      return 0;
    }

    template <typename scalar_type, typename index_type>
    void VectorFit<scalar_type, index_type>::appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const
    {
      for (IdxT q = 0; q < pole_count_; ++q)
      {
        const auto kind = pole_kinds_[static_cast<std::size_t>(q)];
        if (kind == PoleKind::ComplexSecond)
        {
          continue;
        }

        variables.push_back(
            {"w", static_cast<std::size_t>(q), static_cast<std::size_t>(3 * q)});
        if (kind == PoleKind::ComplexFirst)
        {
          variables.push_back(
              {"v", static_cast<std::size_t>(q), static_cast<std::size_t>(3 * (q + 1))});
        }
      }
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::tagDifferentiable()
    {
      for (IdxT j = 0; j < size_; ++j)
      {
        tag_[static_cast<size_t>(j)] = j < output_offset_;
      }
      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
    {
      abs_tol_.setToConst(static_cast<ScalarT>(rel_tol));
      return 0;
    }

    template <typename scalar_type, typename index_type>
    __attribute__((always_inline)) inline int
    VectorFit<scalar_type, index_type>::evaluateInternalResidual(
        const ScalarT* y,
        const ScalarT* yp,
        const ScalarT* ws,
        const ScalarT* wsp,
        ScalarT*       f)
    {
      for (IdxT q = 0; q < pole_count_; ++q)
      {
        const auto state_offset = static_cast<size_t>(3 * q);
        const auto pole         = poles_[static_cast<size_t>(q)];

        if (pole_kinds_[static_cast<size_t>(q)] == PoleKind::Real)
        {
          f[state_offset] = -yp[state_offset]
                            + pole.real() * y[state_offset] + ws[0];
          f[state_offset + 1] = -yp[state_offset + 1]
                                + pole.real() * y[state_offset + 1] + ws[1];
          f[state_offset + 2] = -yp[state_offset + 2]
                                + pole.real() * y[state_offset + 2] + ws[2];
          continue;
        }

        if (q + 1 >= pole_count_)
        {
          return 1;
        }

        const auto v_offset = static_cast<size_t>(3 * (q + 1));
        f[state_offset]     = -yp[state_offset]
                          + pole.real() * y[state_offset]
                          - pole.imag() * y[v_offset] + ws[0];
        f[state_offset + 1] = -yp[state_offset + 1]
                              + pole.real() * y[state_offset + 1]
                              - pole.imag() * y[v_offset + 1] + ws[1];
        f[state_offset + 2] = -yp[state_offset + 2]
                              + pole.real() * y[state_offset + 2]
                              - pole.imag() * y[v_offset + 2] + ws[2];
        f[v_offset] = -yp[v_offset]
                      + pole.imag() * y[state_offset]
                      + pole.real() * y[v_offset];
        f[v_offset + 1] = -yp[v_offset + 1]
                          + pole.imag() * y[state_offset + 1]
                          + pole.real() * y[v_offset + 1];
        f[v_offset + 2] = -yp[v_offset + 2]
                          + pole.imag() * y[state_offset + 2]
                          + pole.real() * y[v_offset + 2];
        ++q;
      }

      const auto out = static_cast<size_t>(output_offset_);
      f[out]         = -y[out]
               + D_[0][0] * ws[0] + D_[0][1] * ws[1] + D_[0][2] * ws[2]
               + E_[0][0] * wsp[0] + E_[0][1] * wsp[1] + E_[0][2] * wsp[2];
      f[out + 1] = -y[out + 1]
                   + D_[1][0] * ws[0] + D_[1][1] * ws[1] + D_[1][2] * ws[2]
                   + E_[1][0] * wsp[0] + E_[1][1] * wsp[1] + E_[1][2] * wsp[2];
      f[out + 2] = -y[out + 2]
                   + D_[2][0] * ws[0] + D_[2][1] * ws[1] + D_[2][2] * ws[2]
                   + E_[2][0] * wsp[0] + E_[2][1] * wsp[1] + E_[2][2] * wsp[2];

      for (IdxT q = 0; q < pole_count_; ++q)
      {
        const auto  state_offset = static_cast<size_t>(3 * q);
        const auto& residue      = residues_[static_cast<size_t>(q)];

        if (pole_kinds_[static_cast<size_t>(q)] == PoleKind::Real)
        {
          f[out] += residue[0][0].real() * y[state_offset]
                    + residue[0][1].real() * y[state_offset + 1]
                    + residue[0][2].real() * y[state_offset + 2];
          f[out + 1] += residue[1][0].real() * y[state_offset]
                        + residue[1][1].real() * y[state_offset + 1]
                        + residue[1][2].real() * y[state_offset + 2];
          f[out + 2] += residue[2][0].real() * y[state_offset]
                        + residue[2][1].real() * y[state_offset + 1]
                        + residue[2][2].real() * y[state_offset + 2];
          continue;
        }

        const auto v_offset  = static_cast<size_t>(3 * (q + 1));
        f[out]              += static_cast<RealT>(2.0)
                  * (residue[0][0].real() * y[state_offset]
                     + residue[0][1].real() * y[state_offset + 1]
                     + residue[0][2].real() * y[state_offset + 2]
                     - residue[0][0].imag() * y[v_offset]
                     - residue[0][1].imag() * y[v_offset + 1]
                     - residue[0][2].imag() * y[v_offset + 2]);
        f[out + 1] += static_cast<RealT>(2.0)
                      * (residue[1][0].real() * y[state_offset]
                         + residue[1][1].real() * y[state_offset + 1]
                         + residue[1][2].real() * y[state_offset + 2]
                         - residue[1][0].imag() * y[v_offset]
                         - residue[1][1].imag() * y[v_offset + 1]
                         - residue[1][2].imag() * y[v_offset + 2]);
        f[out + 2] += static_cast<RealT>(2.0)
                      * (residue[2][0].real() * y[state_offset]
                         + residue[2][1].real() * y[state_offset + 1]
                         + residue[2][2].real() * y[state_offset + 2]
                         - residue[2][0].imag() * y[v_offset]
                         - residue[2][1].imag() * y[v_offset + 1]
                         - residue[2][2].imag() * y[v_offset + 2]);
        ++q;
      }

      return 0;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::evaluateResidual()
    {
      static constexpr auto INPUT_A = VectorFitExternalVariables::input_a;
      static constexpr auto INPUT_B = VectorFitExternalVariables::input_b;
      static constexpr auto INPUT_C = VectorFitExternalVariables::input_c;

      ws_[0] = signals_.template readExternalVariable<INPUT_A>();
      ws_[1] = signals_.template readExternalVariable<INPUT_B>();
      ws_[2] = signals_.template readExternalVariable<INPUT_C>();

      wsp_[0] = ScalarT{0.0};
      wsp_[1] = ScalarT{0.0};
      wsp_[2] = ScalarT{0.0};
      if (signals_.template isDerivativeLinked<INPUT_A>())
      {
        wsp_[0] = signals_.template readExternalVariableDerivative<INPUT_A>();
      }
      if (signals_.template isDerivativeLinked<INPUT_B>())
      {
        wsp_[1] = signals_.template readExternalVariableDerivative<INPUT_B>();
      }
      if (signals_.template isDerivativeLinked<INPUT_C>())
      {
        wsp_[2] = signals_.template readExternalVariableDerivative<INPUT_C>();
      }

      ws_indices_[0] = signals_.template readExternalVariableIndex<INPUT_A>();
      ws_indices_[1] = signals_.template readExternalVariableIndex<INPUT_B>();
      ws_indices_[2] = signals_.template readExternalVariableIndex<INPUT_C>();

      const int status = evaluateInternalResidual(y_.getData(),
                                                  yp_.getData(),
                                                  ws_.data(),
                                                  wsp_.data(),
                                                  f_.getData());
      f_.setDataUpdated();
      return status;
    }

    template <typename scalar_type, typename index_type>
    void VectorFit<scalar_type, index_type>::appendJacobianEntry(
        IdxT  row,
        IdxT  column,
        RealT value)
    {
      J_rows_buffer_[static_cast<size_t>(nnz_)] = row;
      J_cols_buffer_[static_cast<size_t>(nnz_)] = column;
      J_vals_buffer_[static_cast<size_t>(nnz_)] = value;
      ++nnz_;
    }

    template <typename scalar_type, typename index_type>
    int VectorFit<scalar_type, index_type>::evaluateJacobian()
    {
      const auto buffer_size = static_cast<size_t>(12 + 18 * pole_count_);
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[buffer_size];
        J_cols_buffer_ = new IdxT[buffer_size];
        J_vals_buffer_ = new RealT[buffer_size];
      }

      nnz_ = 0;

      for (IdxT q = 0; q < pole_count_; ++q)
      {
        const size_t state_offset = static_cast<size_t>(3 * q);
        const auto   pole         = poles_[static_cast<size_t>(q)];

        if (pole_kinds_[static_cast<size_t>(q)] == PoleKind::Real)
        {
          appendJacobianEntry(residual_indices_[state_offset],
                              variable_indices_[state_offset],
                              pole.real() - alpha_);
          appendJacobianEntry(residual_indices_[state_offset + 1],
                              variable_indices_[state_offset + 1],
                              pole.real() - alpha_);
          appendJacobianEntry(residual_indices_[state_offset + 2],
                              variable_indices_[state_offset + 2],
                              pole.real() - alpha_);

          if (ws_indices_[0] != INVALID_INDEX<IdxT>)
          {
            appendJacobianEntry(residual_indices_[state_offset], ws_indices_[0], 1.0);
          }
          if (ws_indices_[1] != INVALID_INDEX<IdxT>)
          {
            appendJacobianEntry(residual_indices_[state_offset + 1], ws_indices_[1], 1.0);
          }
          if (ws_indices_[2] != INVALID_INDEX<IdxT>)
          {
            appendJacobianEntry(residual_indices_[state_offset + 2], ws_indices_[2], 1.0);
          }
          continue;
        }

        if (q + 1 >= pole_count_)
        {
          return 1;
        }

        const size_t v_offset = static_cast<size_t>(3 * (q + 1));

        appendJacobianEntry(residual_indices_[state_offset],
                            variable_indices_[state_offset],
                            pole.real() - alpha_);
        appendJacobianEntry(residual_indices_[state_offset + 1],
                            variable_indices_[state_offset + 1],
                            pole.real() - alpha_);
        appendJacobianEntry(residual_indices_[state_offset + 2],
                            variable_indices_[state_offset + 2],
                            pole.real() - alpha_);
        appendJacobianEntry(residual_indices_[state_offset],
                            variable_indices_[v_offset],
                            -pole.imag());
        appendJacobianEntry(residual_indices_[state_offset + 1],
                            variable_indices_[v_offset + 1],
                            -pole.imag());
        appendJacobianEntry(residual_indices_[state_offset + 2],
                            variable_indices_[v_offset + 2],
                            -pole.imag());

        appendJacobianEntry(residual_indices_[v_offset],
                            variable_indices_[state_offset],
                            pole.imag());
        appendJacobianEntry(residual_indices_[v_offset + 1],
                            variable_indices_[state_offset + 1],
                            pole.imag());
        appendJacobianEntry(residual_indices_[v_offset + 2],
                            variable_indices_[state_offset + 2],
                            pole.imag());
        appendJacobianEntry(residual_indices_[v_offset],
                            variable_indices_[v_offset],
                            pole.real() - alpha_);
        appendJacobianEntry(residual_indices_[v_offset + 1],
                            variable_indices_[v_offset + 1],
                            pole.real() - alpha_);
        appendJacobianEntry(residual_indices_[v_offset + 2],
                            variable_indices_[v_offset + 2],
                            pole.real() - alpha_);

        if (ws_indices_[0] != INVALID_INDEX<IdxT>)
        {
          appendJacobianEntry(residual_indices_[state_offset], ws_indices_[0], 1.0);
        }
        if (ws_indices_[1] != INVALID_INDEX<IdxT>)
        {
          appendJacobianEntry(residual_indices_[state_offset + 1], ws_indices_[1], 1.0);
        }
        if (ws_indices_[2] != INVALID_INDEX<IdxT>)
        {
          appendJacobianEntry(residual_indices_[state_offset + 2], ws_indices_[2], 1.0);
        }
        ++q;
      }

      const size_t out_a = static_cast<size_t>(output_offset_);
      const size_t out_b = static_cast<size_t>(output_offset_ + 1);
      const size_t out_c = static_cast<size_t>(output_offset_ + 2);

      appendJacobianEntry(residual_indices_[out_a], variable_indices_[out_a], -1.0);
      appendJacobianEntry(residual_indices_[out_b], variable_indices_[out_b], -1.0);
      appendJacobianEntry(residual_indices_[out_c], variable_indices_[out_c], -1.0);

      if (ws_indices_[0] != INVALID_INDEX<IdxT>)
      {
        appendJacobianEntry(residual_indices_[out_a], ws_indices_[0], D_[0][0] + alpha_ * E_[0][0]);
        appendJacobianEntry(residual_indices_[out_b], ws_indices_[0], D_[1][0] + alpha_ * E_[1][0]);
        appendJacobianEntry(residual_indices_[out_c], ws_indices_[0], D_[2][0] + alpha_ * E_[2][0]);
      }
      if (ws_indices_[1] != INVALID_INDEX<IdxT>)
      {
        appendJacobianEntry(residual_indices_[out_a], ws_indices_[1], D_[0][1] + alpha_ * E_[0][1]);
        appendJacobianEntry(residual_indices_[out_b], ws_indices_[1], D_[1][1] + alpha_ * E_[1][1]);
        appendJacobianEntry(residual_indices_[out_c], ws_indices_[1], D_[2][1] + alpha_ * E_[2][1]);
      }
      if (ws_indices_[2] != INVALID_INDEX<IdxT>)
      {
        appendJacobianEntry(residual_indices_[out_a], ws_indices_[2], D_[0][2] + alpha_ * E_[0][2]);
        appendJacobianEntry(residual_indices_[out_b], ws_indices_[2], D_[1][2] + alpha_ * E_[1][2]);
        appendJacobianEntry(residual_indices_[out_c], ws_indices_[2], D_[2][2] + alpha_ * E_[2][2]);
      }

      for (IdxT q = 0; q < pole_count_; ++q)
      {
        const size_t state_offset = static_cast<size_t>(3 * q);
        const auto&  residue      = residues_[static_cast<size_t>(q)];

        if (pole_kinds_[static_cast<size_t>(q)] == PoleKind::Real)
        {
          appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset], residue[0][0].real());
          appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset + 1], residue[0][1].real());
          appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset + 2], residue[0][2].real());
          appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset], residue[1][0].real());
          appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset + 1], residue[1][1].real());
          appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset + 2], residue[1][2].real());
          appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset], residue[2][0].real());
          appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset + 1], residue[2][1].real());
          appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset + 2], residue[2][2].real());
          continue;
        }

        const size_t v_offset = static_cast<size_t>(3 * (q + 1));
        appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset], 2.0 * residue[0][0].real());
        appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset + 1], 2.0 * residue[0][1].real());
        appendJacobianEntry(residual_indices_[out_a], variable_indices_[state_offset + 2], 2.0 * residue[0][2].real());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset], 2.0 * residue[1][0].real());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset + 1], 2.0 * residue[1][1].real());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[state_offset + 2], 2.0 * residue[1][2].real());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset], 2.0 * residue[2][0].real());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset + 1], 2.0 * residue[2][1].real());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[state_offset + 2], 2.0 * residue[2][2].real());

        appendJacobianEntry(residual_indices_[out_a], variable_indices_[v_offset], -2.0 * residue[0][0].imag());
        appendJacobianEntry(residual_indices_[out_a], variable_indices_[v_offset + 1], -2.0 * residue[0][1].imag());
        appendJacobianEntry(residual_indices_[out_a], variable_indices_[v_offset + 2], -2.0 * residue[0][2].imag());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[v_offset], -2.0 * residue[1][0].imag());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[v_offset + 1], -2.0 * residue[1][1].imag());
        appendJacobianEntry(residual_indices_[out_b], variable_indices_[v_offset + 2], -2.0 * residue[1][2].imag());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[v_offset], -2.0 * residue[2][0].imag());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[v_offset + 1], -2.0 * residue[2][1].imag());
        appendJacobianEntry(residual_indices_[out_c], variable_indices_[v_offset + 2], -2.0 * residue[2][2].imag());
        ++q;
      }

      this->constructCoo();
      return 0;
    }
  } // namespace EMT
} // namespace GridKit
