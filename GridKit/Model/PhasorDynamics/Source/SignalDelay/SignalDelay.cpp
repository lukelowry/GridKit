/**
 * @file SignalDelay.cpp
 */

#include "SignalDelayImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /**
       * @brief Analytic Jacobian evaluation
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for SignalDelay..." << std::endl;

        J_.zeroMatrix();

        const size_t max_nnz = delay_ == 0.0 ? 2 : 1;
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

        add_entry(residual_indices_[VALUE_INDEX], variable_indices_[VALUE_INDEX], 1.0);

        if (delay_ == 0.0 && signals_.template isAttached<SignalDelayExternalVariables::INPUT>()
            && signals_.template isLinked<SignalDelayExternalVariables::INPUT>())
        {
          add_entry(residual_indices_[VALUE_INDEX],
                    signals_.template readExternalVariableIndex<SignalDelayExternalVariables::INPUT>(),
                    -1.0);
        }

        J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz);
        return 0;
      }

      // Available template instantiations
      template class SignalDelay<double, long int>;
      template class SignalDelay<double, size_t>;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
