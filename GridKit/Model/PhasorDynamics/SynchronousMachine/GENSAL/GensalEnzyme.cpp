/**
 * @file GensalEnzyme.cpp
 * @author Luke Lowery (lukel@tamu.edu)
 *
 */

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "GensalImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    /**
     * @brief Enzyme Jacobian block sequence for one smoothing mode
     *
     * The bus residual uses no smoothing primitives, so the DhDy block
     * keeps the BusResidual key in both modes.
     *
     * @return int - error code, 0 = success
     */
    template <typename scalar_type, typename index_type>
    template <GridKit::Math::Smoothing M>
    int Gensal<scalar_type, index_type>::evaluateJacobianBlocks()
    {
      using ModelT       = GridKit::PhasorDynamics::Gensal<ScalarT, IdxT>;
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
                                                        wb_.data(),
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

      GridKit::Enzyme::Sparse::DhDy<ModelT, Fn::BusResidual>::eval(this,
                                                                   static_cast<size_t>(bus_->size()),
                                                                   static_cast<size_t>(y_.getSize()),
                                                                   (bus_->getResidualIndices()).data(),
                                                                   (this->getVariableIndices()).data(),
                                                                   y_.getData(),
                                                                   yp_.getData(),
                                                                   wb_.data(),
                                                                   J_rows_buffer_,
                                                                   J_cols_buffer_,
                                                                   J_vals_buffer_,
                                                                   nnz_);

      return 0;
    }

    /**
     * @brief Jacobian evaluation experimental
     *
     * @return int - error code, 0 = success
     */
    template <typename scalar_type, typename index_type>
    int Gensal<scalar_type, index_type>::evaluateJacobian()
    {
      Log::misc() << "Evaluate Jacobian for Gensal..." << std::endl;
      Log::misc() << "Jacobian evaluation is experimental!" << std::endl;

      if (J_rows_buffer_ == nullptr)
      {
        // Reserve space for the dense blocks.
        // The size of the buffer is the sum of maximum capacities of the blocks.
        // Enyme will compute the appropriate nnz from sparsification.
        auto size        = static_cast<size_t>(size_);
        auto bus_size    = static_cast<size_t>(bus_->size());
        auto signal_size = static_cast<size_t>(ws_.size());
        auto buffer_size = 2 * size * size + size * signal_size + 2 * size * bus_size;
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
    template class Gensal<double, long int>;
    template class Gensal<double, size_t>;

  } // namespace PhasorDynamics
} // namespace GridKit
