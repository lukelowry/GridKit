/**
 * @file ReecbEnzyme.cpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Enzyme sparse Jacobian for the REECB electrical-control model.
 */

#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>

#include "ReecbImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Controller
    {
      /**
       * @brief Enzyme Jacobian block sequence for one smoothing mode
       *
       * @return int - error code, 0 = success
       */
      template <typename scalar_type, typename index_type>
      template <GridKit::Math::Smoothing M>
      int Reecb<scalar_type, index_type>::evaluateJacobianBlocks()
      {
        using ModelT       = GridKit::PhasorDynamics::Controller::Reecb<scalar_type, index_type>;
        using Fn           = GridKit::Enzyme::Sparse::MemberFunctions;
        constexpr auto fun = M == GridKit::Math::Smoothing::Piecewise
                                 ? Fn::InternalResidualWithSignalPiecewise
                                 : Fn::InternalResidualWithSignal;

        GridKit::Enzyme::Sparse::DfDy<ModelT, fun>::eval(
            this,
            static_cast<size_t>(f_.getSize()),
            static_cast<size_t>(y_.getSize()),
            this->getResidualIndices().data(),
            this->getVariableIndices().data(),
            y_.getData(),
            yp_.getData(),
            wb_.data(),
            ws_.data(),
            J_rows_buffer_,
            J_cols_buffer_,
            J_vals_buffer_,
            nnz_);

        GridKit::Enzyme::Sparse::DfDyp<ModelT, fun>::eval(
            this,
            static_cast<size_t>(f_.getSize()),
            static_cast<size_t>(y_.getSize()),
            this->getResidualIndices().data(),
            this->getVariableIndices().data(),
            y_.getData(),
            yp_.getData(),
            wb_.data(),
            ws_.data(),
            alpha(),
            J_rows_buffer_,
            J_cols_buffer_,
            J_vals_buffer_,
            nnz_);

        GridKit::Enzyme::Sparse::DfDwb<ModelT, fun>::eval(
            this,
            static_cast<size_t>(f_.getSize()),
            static_cast<size_t>(bus_->size()),
            this->getResidualIndices().data(),
            bus_->getVariableIndices().data(),
            y_.getData(),
            yp_.getData(),
            wb_.data(),
            ws_.data(),
            J_rows_buffer_,
            J_cols_buffer_,
            J_vals_buffer_,
            nnz_);

        GridKit::Enzyme::Sparse::DfDws<ModelT, fun>::eval(
            this,
            static_cast<size_t>(f_.getSize()),
            ws_.size(),
            this->getResidualIndices().data(),
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
       * @brief Assemble the sparse REECB Jacobian with Enzyme.
       *
       * Differentiates the internal residual with respect to state, derivative,
       * terminal-bus, and linked signal variables, then constructs the model
       * COO matrix.
       *
       * @pre allocate() has sized the model and Jacobian index maps.
       * @pre evaluateResidual() has refreshed the current bus/signal values and
       *      signal indices.
       * @pre The containing solver has set the current integration coefficient
       *      and global variable/residual indices.
       */
      template <typename scalar_type, typename index_type>
      int Reecb<scalar_type, index_type>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for Reecb...\n";
        Log::misc() << "Jacobian evaluation is experimental!\n";

        if (J_rows_buffer_ == nullptr)
        {
          const auto size        = static_cast<size_t>(size_);
          const auto bus_size    = static_cast<size_t>(bus_->size());
          const auto signal_size = ws_.size();
          const auto buffer_size = 2 * size * size + size * bus_size + size * signal_size;

          J_rows_buffer_ = new IdxT[buffer_size];
          J_cols_buffer_ = new IdxT[buffer_size];
          J_vals_buffer_ = new RealT[buffer_size];
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

      template class Reecb<double, long int>;
      template class Reecb<double, size_t>;
    } // namespace Controller
  } // namespace PhasorDynamics
} // namespace GridKit
