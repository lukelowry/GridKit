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
          add_entry(residual_indices[zIndex(i)], variable_indices[zIndex(i)], 1.0);
        }

        const auto input_col0 = variable_indices[uIndex(0)];
        const auto state_col0 = memoryStateCount() > 0 ? variable_indices[memoryStateIndex(0)] : static_cast<IdxT>(0);

        approximation_.addOutputJacobianEntries(residual_indices[zIndex(0)],
                                                input_col0,
                                                state_col0,
                                                alpha_,
                                                static_cast<RealT>(-1.0),
                                                add_entry);
        if (memoryStateCount() > 0)
        {
          approximation_.addStateJacobianEntries(residual_indices[memoryStateIndex(0)],
                                                 input_col0,
                                                 state_col0,
                                                 alpha_,
                                                 add_entry);
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
