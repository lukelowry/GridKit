/**
 * @file ConvolutionVec.hpp
 * @brief Declaration of a vector-valued vector-fitting convolution helper
 */

#pragma once

#include <cstddef>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVecData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    template <class ScalarT, typename IdxT>
    class SignalNode;

    namespace Convolution
    {
      template <typename RealT, typename IdxT>
      struct ConvolutionVecData;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /**
       * @brief Square MIMO convolution helper from vector-fitting coefficients.
       *
       * The inherited `y_` vector stores model states in the order
       * all input proxies `u_i`, all outputs `z_i`, then one memory state
       * `x_k` per pole.
       */
      template <class ScalarT, typename IdxT>
      class ConvolutionVec : public Component<ScalarT, IdxT>
      {
        using Component<ScalarT, IdxT>::gridkit_component_id_;
        using Component<ScalarT, IdxT>::alpha_;
        using Component<ScalarT, IdxT>::f_;
        using Component<ScalarT, IdxT>::nnz_;
        using Component<ScalarT, IdxT>::size_;
        using Component<ScalarT, IdxT>::tag_;
        using Component<ScalarT, IdxT>::time_;
        using Component<ScalarT, IdxT>::y_;
        using Component<ScalarT, IdxT>::yp_;
        using Component<ScalarT, IdxT>::wb_;
        using Component<ScalarT, IdxT>::J_;
        using Component<ScalarT, IdxT>::J_rows_buffer_;
        using Component<ScalarT, IdxT>::J_cols_buffer_;
        using Component<ScalarT, IdxT>::J_vals_buffer_;
        using Component<ScalarT, IdxT>::variable_indices_;
        using Component<ScalarT, IdxT>::residual_indices_;

      public:
        using RealT           = typename Component<ScalarT, IdxT>::RealT;
        using model_data_type = ConvolutionVecData<RealT, IdxT>;
        using signal_type     = SignalNode<ScalarT, IdxT>;

        ConvolutionVec();
        ConvolutionVec(const std::vector<signal_type*>& inputs,
                       const std::vector<signal_type*>& outputs,
                       const model_data_type&           data);
        ConvolutionVec(const model_data_type& data);
        ~ConvolutionVec() = default;

        int setGridKitComponentID(IdxT) override final;
        int allocate() override final;
        int verify() const override final;
        int initialize() override final;
        int tagDifferentiable() override final;
        int evaluateResidual() override final;
        int evaluateJacobian() override final;

        /// Attach dynamic input signal nodes, one per input component.
        int attachInputSignals(const std::vector<signal_type*>& inputs);

        /// Assign dynamic output signal nodes, one per output component.
        int assignOutputSignals(const std::vector<signal_type*>& outputs);

        /// Return the square input/output vector dimension.
        auto dimension() const -> size_t
        {
          return dimension_;
        }

        /// Return the number of vector-fitting modes.
        auto modeCount() const -> size_t
        {
          return poles_.size();
        }

      public:
        __attribute__((always_inline)) inline int evaluateInternalResidual(ScalarT*, ScalarT*, ScalarT*, ScalarT*, ScalarT*);

      private:
        void initializeParameters(const model_data_type& data);
        void linkOutputSignals();

        auto uIndex(size_t i) const -> size_t
        {
          return i;
        }

        auto zIndex(size_t i) const -> size_t
        {
          return dimension_ + i;
        }

        auto xIndex(size_t k) const -> size_t
        {
          return static_cast<size_t>(2) * dimension_ + k;
        }

        auto matrixIndex(size_t row, size_t col) const -> size_t
        {
          return row * dimension_ + col;
        }

        auto modeVectorIndex(size_t mode, size_t component) const -> size_t
        {
          return mode * dimension_ + component;
        }

      private:
        size_t dimension_{0};

        std::vector<RealT> d_;
        std::vector<RealT> e_;
        std::vector<RealT> poles_;
        std::vector<RealT> input_couplings_;
        std::vector<RealT> output_residues_;
        std::vector<RealT> u0_;
        std::vector<RealT> up0_;

        std::vector<signal_type*> input_signals_;
        std::vector<signal_type*> output_signals_;

        // Local copy of signal values
        std::vector<ScalarT> ws_;
      };

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
