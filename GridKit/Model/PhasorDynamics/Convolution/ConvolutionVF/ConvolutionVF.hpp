/**
 * @file ConvolutionVF.hpp
 * @brief Declaration of a vector-fitting convolution helper
 */

#pragma once

#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVFData.hpp>

// Forward declarations
namespace GridKit
{
  namespace PhasorDynamics
  {
    template <class ScalarT, typename IdxT>
    class SignalNode;

    namespace Convolution
    {
      template <typename RealT, typename IdxT>
      struct ConvolutionVFData;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /// Internal variables of a `ConvolutionVF`
      enum class ConvolutionVFInternalVariables : size_t
      {
        Z, ///< Convolution output
        MAXIMUM,
      };

      /// External variables of a `ConvolutionVF`
      enum class ConvolutionVFExternalVariables : size_t
      {
        U, ///< Convolution input
        MAXIMUM,
      };

      /**
       * @brief Scalar convolution helper from vector-fitting coefficients.
       *
       * The inherited `y_` vector stores model states in the order
       * `u`, `z`, then one memory state `x_n` per pole.
       */
      template <class ScalarT, typename IdxT>
      class ConvolutionVF : public Component<ScalarT, IdxT>
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
        using model_data_type = ConvolutionVFData<RealT, IdxT>;
        using signal_type     = SignalNode<ScalarT, IdxT>;

        ConvolutionVF();
        ConvolutionVF(signal_type* input, signal_type* output, const model_data_type& data);
        ConvolutionVF(const model_data_type& data);
        ~ConvolutionVF() = default;

        int setGridKitComponentID(IdxT) override final;
        int allocate() override final;
        int verify() const override final;
        int initialize() override final;
        int tagDifferentiable() override final;
        int evaluateResidual() override final;
        int evaluateJacobian() override final;

        /// Get the `ComponentSignals` from this `ConvolutionVF`
        auto getSignals()
            -> ComponentSignals<ScalarT,
                                IdxT,
                                ConvolutionVFInternalVariables,
                                ConvolutionVFExternalVariables>&
        {
          return signals_;
        }

      public:
        __attribute__((always_inline)) inline int evaluateInternalResidual(ScalarT*, ScalarT*, ScalarT*, ScalarT*, ScalarT*);

      private:
        void initializeParameters(const model_data_type& data);

      private:
        static constexpr size_t U_INDEX  = 0;
        static constexpr size_t Z_INDEX  = 1;
        static constexpr size_t X_OFFSET = 2;

        RealT d_{0.0};
        RealT e_{0.0};
        RealT u0_{0.0};
        RealT up0_{0.0};

        std::vector<RealT> poles_;
        std::vector<RealT> residues_;

        /// Component signal extension
        ComponentSignals<ScalarT,
                         IdxT,
                         ConvolutionVFInternalVariables,
                         ConvolutionVFExternalVariables>
            signals_;

        // Local copy of signal value
        std::vector<ScalarT> ws_;
      };

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
