/**
 * @file Esdc1aEnzyme.cpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Enzyme sparse Jacobian for the ESDC1A exciter model.
 */

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "Esdc1aImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Exciter
    {
      /**
       * @brief Enzyme Jacobian block sequence for one smoothing mode
       *
       * @return Zero on success.
       */
      template <typename scalar_type, typename index_type>
      template <GridKit::Math::Smoothing M>
      int Esdc1a<scalar_type, index_type>::evaluateJacobianBlocks()
      {
        using ModelT       = GridKit::PhasorDynamics::Exciter::Esdc1a<ScalarT, IdxT>;
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

        return 0;
      }

      /**
       * @brief Assemble the sparse ESDC1A component Jacobian with Enzyme.
       *
       * Differentiates the internal residual with respect to internal states,
       * state derivatives, terminal-bus variables, and linked signal values,
       * then assembles the resulting entries in COO form.
       *
       * @pre allocate() has completed.
       * @pre evaluateResidual() has refreshed the interface buffers at the
       *      current state.
       * @pre Solver alpha and global variable and residual indices are set.
       */
      template <typename scalar_type, typename index_type>
      int Esdc1a<scalar_type, index_type>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for Esdc1a..." << std::endl;
        Log::misc() << "Jacobian evaluation is experimental!" << std::endl;

        if (J_rows_buffer_ == nullptr)
        {
          auto size        = static_cast<size_t>(size_);
          auto bus_size    = static_cast<size_t>(bus_->size());
          auto signal_size = ws_.size();
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

      template class Esdc1a<double, long int>;
      template class Esdc1a<double, size_t>;
    } // namespace Exciter
  } // namespace PhasorDynamics
} // namespace GridKit
