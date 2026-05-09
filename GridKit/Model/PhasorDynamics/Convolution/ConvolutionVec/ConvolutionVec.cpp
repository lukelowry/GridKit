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

        if (J_rows_buffer_ == nullptr && jacobianEntryCapacity() > 0)
        {
          Log::error() << "ConvolutionVec: allocate must be called before evaluateJacobian\n";
          return 1;
        }

        const auto real_mode_count    = realModeCount();
        const auto complex_pair_count = complexPairCount();

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

          for (size_t k = 0; k < real_mode_count; ++k)
          {
            add_entry(residual_indices[zIndex(i)],
                      variable_indices[xIndex(k)],
                      -output_residues_[modeVectorIndex(k, i)]);
          }
          for (size_t pair = 0; pair < complex_pair_count; ++pair)
          {
            add_entry(residual_indices[zIndex(i)],
                      variable_indices[complexRealStateIndex(pair)],
                      -static_cast<RealT>(2.0) * complex_output_residues_real_[modeVectorIndex(pair, i)]);
            add_entry(residual_indices[zIndex(i)],
                      variable_indices[complexImagStateIndex(pair)],
                      static_cast<RealT>(2.0) * complex_output_residues_imag_[modeVectorIndex(pair, i)]);
          }
        }

        for (size_t k = 0; k < real_mode_count; ++k)
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

        for (size_t pair = 0; pair < complex_pair_count; ++pair)
        {
          const auto real_state_index = complexRealStateIndex(pair);
          const auto imag_state_index = complexImagStateIndex(pair);

          // f[xr_q] = -xr_q' + br_q*u + ar_q*xr_q - ai_q*xi_q
          for (size_t i = 0; i < dimension_; ++i)
          {
            add_entry(residual_indices[real_state_index],
                      variable_indices[uIndex(i)],
                      complex_input_couplings_real_[modeVectorIndex(pair, i)]);
          }
          add_entry(residual_indices[real_state_index],
                    variable_indices[real_state_index],
                    complex_pole_real_[pair] - alpha_);
          add_entry(residual_indices[real_state_index],
                    variable_indices[imag_state_index],
                    -complex_pole_imag_[pair]);

          // f[xi_q] = -xi_q' + bi_q*u + ai_q*xr_q + ar_q*xi_q
          for (size_t i = 0; i < dimension_; ++i)
          {
            add_entry(residual_indices[imag_state_index],
                      variable_indices[uIndex(i)],
                      complex_input_couplings_imag_[modeVectorIndex(pair, i)]);
          }
          add_entry(residual_indices[imag_state_index],
                    variable_indices[real_state_index],
                    complex_pole_imag_[pair]);
          add_entry(residual_indices[imag_state_index],
                    variable_indices[imag_state_index],
                    complex_pole_real_[pair] - alpha_);
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
