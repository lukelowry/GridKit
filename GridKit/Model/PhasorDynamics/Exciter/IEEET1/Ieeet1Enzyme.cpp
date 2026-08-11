/**
 * @file Ieeet1Enzyme.cpp
 * @author Nicholson Koukpaizan (koukpaizannk@ornl.gov)
 *
 */

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "Ieeet1Impl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Exciter
    {
      /**
       * @brief Enzyme Jacobian block sequence for one smoothing mode
       *
       * @return int - error code, 0 = success
       */
      template <typename scalar_type, typename index_type>
      template <GridKit::Math::Smoothing M>
      int Ieeet1<scalar_type, index_type>::evaluateJacobianBlocks()
      {
        using ModelT       = GridKit::PhasorDynamics::Exciter::Ieeet1<ScalarT, IdxT>;
        using Fn           = GridKit::Enzyme::Sparse::MemberFunctions;
        constexpr auto fun = M == GridKit::Math::Smoothing::Piecewise
                                 ? Fn::InternalResidualWithSignalPiecewise
                                 : Fn::InternalResidualWithSignal;

        GridKit::Enzyme::Sparse::DfDy<ModelT, fun>::eval(this,
                                                         static_cast<size_t>(f_.getSize()),
                                                         static_cast<size_t>(y_.getSize()),
                                                         (this->getResidualIndices()).data(),
                                                         (this->getVariableIndices()).data(),
                                                         y_.getData(),
                                                         yp_.getData(),
                                                         wb_.data(),
                                                         ws_.data(),
                                                         J_rows_buffer_,
                                                         J_cols_buffer_,
                                                         J_vals_buffer_,
                                                         nnz_);

        GridKit::Enzyme::Sparse::DfDyp<ModelT, fun>::eval(this,
                                                          static_cast<size_t>(f_.getSize()),
                                                          static_cast<size_t>(y_.getSize()),
                                                          (this->getResidualIndices()).data(),
                                                          (this->getVariableIndices()).data(),
                                                          y_.getData(),
                                                          yp_.getData(),
                                                          wb_.data(),
                                                          ws_.data(),
                                                          alpha(),
                                                          J_rows_buffer_,
                                                          J_cols_buffer_,
                                                          J_vals_buffer_,
                                                          nnz_);

        GridKit::Enzyme::Sparse::DfDwb<ModelT, fun>::eval(this,
                                                          static_cast<size_t>(f_.getSize()),
                                                          static_cast<size_t>(bus_->size()),
                                                          (this->getResidualIndices()).data(),
                                                          (bus_->getVariableIndices()).data(),
                                                          y_.getData(),
                                                          yp_.getData(),
                                                          bus_->y().getData(),
                                                          ws_.data(),
                                                          J_rows_buffer_,
                                                          J_cols_buffer_,
                                                          J_vals_buffer_,
                                                          nnz_);

        GridKit::Enzyme::Sparse::DfDws<ModelT, fun>::eval(this,
                                                          static_cast<size_t>(f_.getSize()),
                                                          ws_.size(),
                                                          (this->getResidualIndices()).data(),
                                                          ws_indices_.data(),
                                                          y_.getData(),
                                                          yp_.getData(),
                                                          wb_.data(),
                                                          ws_.data(),
                                                          J_rows_buffer_,
                                                          J_cols_buffer_,
                                                          J_vals_buffer_,
                                                          nnz_);

        return 0;
      }

      /**
       * @brief Jacobian evaluation not implemented yet
       *
       * @return int - error code, 0 = success
       */
      template <class scalar_type, typename index_type>
      int Ieeet1<scalar_type, index_type>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for Ieeet1..." << std::endl;
        Log::misc() << "Jacobian evaluation is experimental!" << std::endl;

        if (J_rows_buffer_ == nullptr)
        {
          // Reserve space for the dense blocks.
          // The size of the buffer is the sum of maximum capacities of the blocks.
          // Enyme will compute the appropriate nnz from sparsification.
          auto size        = static_cast<size_t>(size_);
          auto bus_size    = static_cast<size_t>(bus_->size());
          auto signal_size = static_cast<size_t>(ws_.size());
          auto buffer_size = 2 * size * size + size * bus_size + size * signal_size;
          J_rows_buffer_   = new IdxT[buffer_size];
          J_cols_buffer_   = new IdxT[buffer_size];
          J_vals_buffer_   = new RealT[buffer_size];
        }

        nnz_ = 0;

        if (GridKit::Math::SMOOTHING_MODE == GridKit::Math::Smoothing::Piecewise)
        {
          evaluateJacobianBlocks<GridKit::Math::Smoothing::Piecewise>();
        }
        else
        {
          evaluateJacobianBlocks<GridKit::Math::Smoothing::Smooth>();
        }

        this->constructCoo();

        return 0;
      }

      // Available template instantiations
      template class Ieeet1<double, long int>;
      template class Ieeet1<double, size_t>;

    } // namespace Exciter
  } // namespace PhasorDynamics
} // namespace GridKit
