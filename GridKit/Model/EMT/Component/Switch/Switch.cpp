#include <stdexcept>

#include "SwitchImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::evaluateJacobian()
    {
      // Three residual rows over the union of the series-current, terminal-1
      // and terminal-2 columns, plus the six bus-injection entries. The switch
      // position changes values only; the entries and their order are the same
      // in both positions.
      static constexpr IdxT JACOBIAN_CAPACITY{15};
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_cols_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_vals_buffer_ = new RealT[JACOBIAN_CAPACITY];
      }

      const RealT open   = openCommand();
      const RealT closed = RealT{1.0} - open;

      nnz_ = 0;
      for (IdxT phase = 0; phase < 3; ++phase)
      {
        J_rows_buffer_[nnz_] = residual_indices_[phase];
        J_cols_buffer_[nnz_] = variable_indices_[phase];
        J_vals_buffer_[nnz_] = open;
        ++nnz_;
        J_rows_buffer_[nnz_] = residual_indices_[phase];
        J_cols_buffer_[nnz_] = bus1_->getVariableIndex(phase);
        J_vals_buffer_[nnz_] = -closed;
        ++nnz_;
        J_rows_buffer_[nnz_] = residual_indices_[phase];
        J_cols_buffer_[nnz_] = bus2_->getVariableIndex(phase);
        J_vals_buffer_[nnz_] = closed;
        ++nnz_;
        J_rows_buffer_[nnz_] = bus1_->getResidualIndex(phase);
        J_cols_buffer_[nnz_] = variable_indices_[phase];
        J_vals_buffer_[nnz_] = RealT{-1.0};
        ++nnz_;
        J_rows_buffer_[nnz_] = bus2_->getResidualIndex(phase);
        J_cols_buffer_[nnz_] = variable_indices_[phase];
        J_vals_buffer_[nnz_] = RealT{1.0};
        ++nnz_;
      }

      if (nnz_ > JACOBIAN_CAPACITY)
      {
        throw std::runtime_error(
            "EMT::Switch: Jacobian entries exceed the reserved capacity");
      }
      this->constructCoo();
      return 0;
    }

    template class Switch<double, long int>;
    template class Switch<double, size_t>;
  } // namespace EMT
} // namespace GridKit
