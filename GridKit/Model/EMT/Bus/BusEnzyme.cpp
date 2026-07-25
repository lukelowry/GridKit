#include "BusImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::evaluateJacobian()
    {
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[9];
        J_cols_buffer_ = new IdxT[9];
        J_vals_buffer_ = new RealT[9];
      }

      nnz_              = 0;
      J_rows_buffer_[0] = residual_indices_[0];
      J_cols_buffer_[0] = variable_indices_[0];
      J_vals_buffer_[0] = RealT{0.0};
      J_rows_buffer_[1] = residual_indices_[0];
      J_cols_buffer_[1] = variable_indices_[1];
      J_vals_buffer_[1] = RealT{0.0};
      J_rows_buffer_[2] = residual_indices_[0];
      J_cols_buffer_[2] = variable_indices_[2];
      J_vals_buffer_[2] = RealT{0.0};
      J_rows_buffer_[3] = residual_indices_[1];
      J_cols_buffer_[3] = variable_indices_[0];
      J_vals_buffer_[3] = RealT{0.0};
      J_rows_buffer_[4] = residual_indices_[1];
      J_cols_buffer_[4] = variable_indices_[1];
      J_vals_buffer_[4] = RealT{0.0};
      J_rows_buffer_[5] = residual_indices_[1];
      J_cols_buffer_[5] = variable_indices_[2];
      J_vals_buffer_[5] = RealT{0.0};
      J_rows_buffer_[6] = residual_indices_[2];
      J_cols_buffer_[6] = variable_indices_[0];
      J_vals_buffer_[6] = RealT{0.0};
      J_rows_buffer_[7] = residual_indices_[2];
      J_cols_buffer_[7] = variable_indices_[1];
      J_vals_buffer_[7] = RealT{0.0};
      J_rows_buffer_[8] = residual_indices_[2];
      J_cols_buffer_[8] = variable_indices_[2];
      J_vals_buffer_[8] = RealT{0.0};
      nnz_              = 9;
      this->constructCoo();
      return 0;
    }

    template class Bus<double, long int>;
    template class Bus<double, size_t>;
  } // namespace EMT
} // namespace GridKit
