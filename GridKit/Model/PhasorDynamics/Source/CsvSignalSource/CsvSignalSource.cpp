/**
 * @file CsvSignalSource.cpp
 */

#include "CsvSignalSourceImpl.hpp"

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
      int CsvSignalSource<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for CsvSignalSource..." << std::endl;

        J_.zeroMatrix();

        if (J_rows_buffer_ == nullptr)
        {
          J_rows_buffer_ = new IdxT[1];
          J_cols_buffer_ = new IdxT[1];
          J_vals_buffer_ = new RealT[1];
        }

        J_rows_buffer_[0] = residual_indices_[VALUE_INDEX];
        J_cols_buffer_[0] = variable_indices_[VALUE_INDEX];
        J_vals_buffer_[0] = 1.0;

        J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, 1);
        return 0;
      }

      // Available template instantiations
      template class CsvSignalSource<double, long int>;
      template class CsvSignalSource<double, size_t>;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
