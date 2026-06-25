#pragma once

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LoadRL<scalar_type, index_type>::evaluateJacobian()
    {
      readBusVoltage();
      J_.zeroMatrix();

      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[9];
        J_cols_buffer_ = new IdxT[9];
        J_vals_buffer_ = new RealT[9];
      }

      GridKit::Enzyme::Sparse::DfDy<GridKit::EMT::LoadRL<ScalarT, IdxT>,
                                    GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                    ScalarT,
                                    IdxT>::eval(this,
                                                f_.size(),
                                                y_.size(),
                                                this->getResidualIndices().data(),
                                                this->getVariableIndices().data(),
                                                y_.data(),
                                                yp_.data(),
                                                wb_.data(),
                                                alpha_,
                                                J_rows_buffer_,
                                                J_cols_buffer_,
                                                J_vals_buffer_,
                                                J_);

      GridKit::Enzyme::Sparse::DfDwb<GridKit::EMT::LoadRL<ScalarT, IdxT>,
                                     GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                     ScalarT,
                                     IdxT>::eval(this,
                                                 f_.size(),
                                                 static_cast<std::size_t>(bus_->size()),
                                                 this->getResidualIndices().data(),
                                                 bus_->getVariableIndices().data(),
                                                 y_.data(),
                                                 yp_.data(),
                                                 wb_.data(),
                                                 J_rows_buffer_,
                                                 J_cols_buffer_,
                                                 J_vals_buffer_,
                                                 J_);

      GridKit::Enzyme::Sparse::DhDy<GridKit::EMT::LoadRL<ScalarT, IdxT>,
                                    GridKit::Enzyme::Sparse::MemberFunctions::BusResidual,
                                    ScalarT,
                                    IdxT>::eval(this,
                                                static_cast<std::size_t>(bus_->size()),
                                                y_.size(),
                                                bus_->getResidualIndices().data(),
                                                this->getVariableIndices().data(),
                                                y_.data(),
                                                yp_.data(),
                                                wb_.data(),
                                                J_rows_buffer_,
                                                J_cols_buffer_,
                                                J_vals_buffer_,
                                                J_);

      return 0;
    }
  } // namespace EMT
} // namespace GridKit
#endif
