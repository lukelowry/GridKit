/**
 * @file ConvolutionVec.cpp
 */

#include "ConvolutionVecImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /**
       * @brief Analytic Jacobian evaluation
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for ConvolutionVec..." << std::endl;

        J_.zeroMatrix();

        const auto mode_count = poles_.size();
        const auto max_nnz    = static_cast<size_t>(3) * dimension_
                             + dimension_ * dimension_
                             + static_cast<size_t>(2) * dimension_ * mode_count
                             + mode_count;

        if (J_rows_buffer_ == nullptr && max_nnz > 0)
        {
          J_rows_buffer_ = new IdxT[max_nnz];
          J_cols_buffer_ = new IdxT[max_nnz];
          J_vals_buffer_ = new RealT[max_nnz];
        }

        IdxT nnz       = 0;
        auto add_entry = [&](IdxT row, IdxT col, RealT value)
        {
          J_rows_buffer_[nnz] = row;
          J_cols_buffer_[nnz] = col;
          J_vals_buffer_[nnz] = value;
          ++nnz;
        };

        const auto* residual_indices = this->getResidualIndices().data();
        const auto* variable_indices = this->getVariableIndices().data();

        for (size_t i = 0; i < dimension_; ++i)
        {
          // f[u_i] = u_i - U_i
          add_entry(residual_indices[uIndex(i)], variable_indices[uIndex(i)], 1.0);
          if (i < input_signals_.size() && input_signals_[i] != nullptr && input_signals_[i]->linked())
          {
            add_entry(residual_indices[uIndex(i)], input_signals_[i]->getVariableIndex(), -1.0);
          }
        }

        for (size_t i = 0; i < dimension_; ++i)
        {
          // f[z_i] = z_i - row_i(D)*u - row_i(E)*u' - sum(c[k,i]*x[k])
          for (size_t j = 0; j < dimension_; ++j)
          {
            const auto coefficient_index = matrixIndex(i, j);
            add_entry(residual_indices[zIndex(i)],
                      variable_indices[uIndex(j)],
                      -d_[coefficient_index] - alpha_ * e_[coefficient_index]);
          }

          add_entry(residual_indices[zIndex(i)], variable_indices[zIndex(i)], 1.0);

          for (size_t k = 0; k < mode_count; ++k)
          {
            add_entry(residual_indices[zIndex(i)],
                      variable_indices[xIndex(k)],
                      -output_residues_[modeVectorIndex(k, i)]);
          }
        }

        for (size_t k = 0; k < mode_count; ++k)
        {
          // f[x_k] = -x_k' + b_k*u + p_k*x_k
          for (size_t i = 0; i < dimension_; ++i)
          {
            add_entry(residual_indices[xIndex(k)],
                      variable_indices[uIndex(i)],
                      input_couplings_[modeVectorIndex(k, i)]);
          }

          add_entry(residual_indices[xIndex(k)], variable_indices[xIndex(k)], poles_[k] - alpha_);
        }

        J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);

        return 0;
      }

      // Available template instantiations
      template class ConvolutionVec<double, long int>;
      template class ConvolutionVec<double, size_t>;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
