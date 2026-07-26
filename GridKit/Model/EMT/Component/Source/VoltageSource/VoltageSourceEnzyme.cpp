#include <stdexcept>

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "VoltageSourceImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::evaluateJacobian()
    {
      using Function = Enzyme::Sparse::MemberFunctions;
      using ModelT   = VoltageSource<ScalarT, IdxT>;

      // Upper bound on emitted entries:
      // 6 residual rows over 6 local and 3 bus columns for DfDy and DfDyp,
      // plus the 3 hand-written bus-injection entries.
      static constexpr IdxT JACOBIAN_CAPACITY{93};
      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_cols_buffer_ = new IdxT[JACOBIAN_CAPACITY];
        J_vals_buffer_ = new RealT[JACOBIAN_CAPACITY];
      }

      nnz_ = 0;
      Enzyme::Sparse::DfDy<ModelT, Function::InternalResidual>::eval(
          this, 6, 6, residual_indices_.data(), variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);
      Enzyme::Sparse::DfDyp<ModelT, Function::InternalResidual>::eval(
          this, 6, 6, residual_indices_.data(), variable_indices_.data(), y_.getData(), yp_.getData(), wb_.data(), alpha_, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);
      Enzyme::Sparse::DfDwb<ModelT, Function::InternalResidual>::eval(
          this, 6, 3, residual_indices_.data(), bus_->getVariableIndices().data(), y_.getData(), yp_.getData(), wb_.data(), J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, nnz_);

      J_rows_buffer_[nnz_]  = bus_->getResidualIndex(0);
      J_cols_buffer_[nnz_]  = variable_indices_[0];
      const RealT kcl_scale = bus_->differentiatedKCL() ? alpha_ : RealT{1.0};

      J_vals_buffer_[nnz_] = kcl_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus_->getResidualIndex(1);
      J_cols_buffer_[nnz_] = variable_indices_[1];
      J_vals_buffer_[nnz_] = kcl_scale;
      ++nnz_;
      J_rows_buffer_[nnz_] = bus_->getResidualIndex(2);
      J_cols_buffer_[nnz_] = variable_indices_[2];
      J_vals_buffer_[nnz_] = kcl_scale;
      ++nnz_;

      if (nnz_ > JACOBIAN_CAPACITY)
      {
        throw std::runtime_error(
            "EMT::VoltageSource: Jacobian entries exceed the reserved capacity");
      }
      this->constructCoo();
      return 0;
    }

    template class VoltageSource<double, long int>;
    template class VoltageSource<double, size_t>;
  } // namespace EMT
} // namespace GridKit
