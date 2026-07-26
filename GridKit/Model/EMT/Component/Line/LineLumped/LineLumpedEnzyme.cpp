#include <stdexcept>

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "LineLumpedImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::evaluateJacobian()
    {
      using Function = Enzyme::Sparse::MemberFunctions;
      using ModelT   = LineLumped<ScalarT, IdxT>;

      // Upper bound on emitted entries:
      // 9 residual rows over 9 local and 6 bus columns for both DfDy and DfDyp,
      // plus the 6 hand-written bus-injection entries.
      static constexpr IdxT JACOBIAN_CAPACITY{282};
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_cols_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_vals_buffer_ = new RealT[JACOBIAN_CAPACITY];
      }

      bus_variable_indices_[0] = bus1_->getVariableIndex(0);
      bus_variable_indices_[1] = bus1_->getVariableIndex(1);
      bus_variable_indices_[2] = bus1_->getVariableIndex(2);
      bus_variable_indices_[3] = bus2_->getVariableIndex(0);
      bus_variable_indices_[4] = bus2_->getVariableIndex(1);
      bus_variable_indices_[5] = bus2_->getVariableIndex(2);

      nnz_ = 0;
      Enzyme::Sparse::DfDy<ModelT, Function::InternalResidualWithBusDerivative>::eval(
          this, 9, 9, residual_indices_.data(), variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), wbp_.data(), J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);
      Enzyme::Sparse::DfDyp<ModelT, Function::InternalResidualWithBusDerivative>::eval(
          this, 9, 9, residual_indices_.data(), variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), wbp_.data(), alpha_, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);
      Enzyme::Sparse::DfDwb<ModelT, Function::InternalResidualWithBusDerivative>::eval(
          this, 9, 6, residual_indices_.data(), bus_variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), wbp_.data(), J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);
      Enzyme::Sparse::DfDwbp<ModelT, Function::InternalResidualWithBusDerivative>::eval(
          this, 9, 6, residual_indices_.data(), bus_variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), wbp_.data(), alpha_, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);

      const bool  bus1_differentiated = bus1_->differentiatedKCL();
      const bool  bus2_differentiated = bus2_->differentiatedKCL();
      const RealT bus1_current_scale  = bus1_differentiated ? alpha_ : RealT{1.0};
      const RealT bus2_current_scale  = bus2_differentiated ? alpha_ : RealT{1.0};
      const RealT bus1_shunt_scale    = bus1_differentiated ? RealT{0.0} : RealT{1.0};
      const RealT bus2_shunt_scale    = bus2_differentiated ? RealT{0.0} : RealT{1.0};

      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(0);
      J_cols_buffer_[nnz_] = variable_indices_[0];
      J_vals_buffer_[nnz_] = -bus1_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(0);
      J_cols_buffer_[nnz_] = variable_indices_[3];
      J_vals_buffer_[nnz_] = bus1_shunt_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(1);
      J_cols_buffer_[nnz_] = variable_indices_[1];
      J_vals_buffer_[nnz_] = -bus1_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(1);
      J_cols_buffer_[nnz_] = variable_indices_[4];
      J_vals_buffer_[nnz_] = bus1_shunt_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(2);
      J_cols_buffer_[nnz_] = variable_indices_[2];
      J_vals_buffer_[nnz_] = -bus1_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus1_->getResidualIndex(2);
      J_cols_buffer_[nnz_] = variable_indices_[5];
      J_vals_buffer_[nnz_] = bus1_shunt_scale;
      ++nnz_;

      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(0);
      J_cols_buffer_[nnz_] = variable_indices_[0];
      J_vals_buffer_[nnz_] = bus2_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(0);
      J_cols_buffer_[nnz_] = variable_indices_[6];
      J_vals_buffer_[nnz_] = bus2_shunt_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(1);
      J_cols_buffer_[nnz_] = variable_indices_[1];
      J_vals_buffer_[nnz_] = bus2_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(1);
      J_cols_buffer_[nnz_] = variable_indices_[7];
      J_vals_buffer_[nnz_] = bus2_shunt_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(2);
      J_cols_buffer_[nnz_] = variable_indices_[2];
      J_vals_buffer_[nnz_] = bus2_current_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus2_->getResidualIndex(2);
      J_cols_buffer_[nnz_] = variable_indices_[8];
      J_vals_buffer_[nnz_] = bus2_shunt_scale;
      ++nnz_;

      if (nnz_ > JACOBIAN_CAPACITY)
      {
        throw std::runtime_error(
            "EMT::LineLumped: Jacobian entries exceed the reserved capacity");
      }
      this->constructCoo();
      return 0;
    }

    template class LineLumped<double, long int>;
    template class LineLumped<double, size_t>;
  } // namespace EMT
} // namespace GridKit
