/**
 * @file ConvolutionVF.cpp
 */

#include "ConvolutionVFImpl.hpp"

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
      int ConvolutionVF<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for ConvolutionVF..." << std::endl;

        J_.zeroMatrix();
        const auto mode_count = static_cast<size_t>(size_) - X_OFFSET;
        const auto max_nnz    = static_cast<size_t>(4) + static_cast<size_t>(3) * mode_count;

        if (J_rows_buffer_ == nullptr)
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

        // f[u] = u - U
        add_entry(residual_indices[U_INDEX], variable_indices[U_INDEX], 1.0);
        if (ws_indices_[0] != INVALID_INDEX<IdxT>)
        {
          add_entry(residual_indices[U_INDEX], ws_indices_[0], -1.0);
        }

        // f[z] = z - d*u - e*u' - sum(r[n]*x[n])
        add_entry(residual_indices[Z_INDEX], variable_indices[U_INDEX], -d_ - alpha_ * e_);
        add_entry(residual_indices[Z_INDEX], variable_indices[Z_INDEX], 1.0);
        for (size_t n = 0; n < mode_count; ++n)
        {
          add_entry(residual_indices[Z_INDEX], variable_indices[X_OFFSET + n], -residues_[n]);
        }

        // f[x[n]] = -x'[n] + u + p[n]*x[n]
        for (size_t n = 0; n < mode_count; ++n)
        {
          const auto state_index = X_OFFSET + n;

          add_entry(residual_indices[state_index], variable_indices[U_INDEX], 1.0);
          add_entry(residual_indices[state_index], variable_indices[state_index], poles_[n] - alpha_);
        }

        J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);

        return 0;
      }

      // Available template instantiations
      template class ConvolutionVF<double, long int>;
      template class ConvolutionVF<double, size_t>;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
