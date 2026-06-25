#pragma once

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    int VoltageSource<scalar_type, index_type, N>::evaluateJacobian()
    {
      readBusVoltage();

      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[N * N];
        J_cols_buffer_ = new IdxT[N * N];
        J_vals_buffer_ = new RealT[N * N];
      }

      GridKit::Enzyme::Sparse::DhDwb<GridKit::EMT::VoltageSource<ScalarT, IdxT, N>,
                                     GridKit::Enzyme::Sparse::MemberFunctions::BusResidual,
                                     ScalarT,
                                     IdxT>::eval(this,
                                                 N,
                                                 N,
                                                 bus_->getResidualIndices().data(),
                                                 bus_->getVariableIndices().data(),
                                                 y_.data(),
                                                 yp_.data(),
                                                 wb_.data(),
                                                 J_rows_buffer_,
                                                 J_cols_buffer_,
                                                 J_vals_buffer_,
                                                 bus_->getJacobian());

      return 0;
    }
  } // namespace EMT
} // namespace GridKit
#endif
