/**
 * @file SignalDelay.hpp
 * @brief Declaration of a signal delay
 */

#pragma once

#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/History.hpp>
#include <GridKit/Model/PhasorDynamics/Source/SignalDelay/SignalDelayData.hpp>

// Forward declarations
namespace GridKit
{
  namespace PhasorDynamics
  {
    template <class ScalarT, typename IdxT>
    class SignalNode;

    namespace Source
    {
      template <typename RealT, typename IdxT>
      struct SignalDelayData;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /// Internal variables of a `SignalDelay`
      enum class SignalDelayInternalVariables : size_t
      {
        VALUE, ///< Delayed signal value
        MAXIMUM,
      };

      /// External variables of a `SignalDelay`
      enum class SignalDelayExternalVariables : size_t
      {
        INPUT, ///< Input signal value
        MAXIMUM,
      };

      /**
       * @brief Signal delay using accepted-step history.
       *
       * The inherited `y_` vector stores one algebraic output variable,
       * which is assigned to a `SignalNode` for use by other components.
       */
      template <class ScalarT, typename IdxT>
      class SignalDelay : public Component<ScalarT, IdxT>
      {
        using Component<ScalarT, IdxT>::gridkit_component_id_;
        using Component<ScalarT, IdxT>::alpha_;
        using Component<ScalarT, IdxT>::f_;
        using Component<ScalarT, IdxT>::size_;
        using Component<ScalarT, IdxT>::tag_;
        using Component<ScalarT, IdxT>::time_;
        using Component<ScalarT, IdxT>::y_;
        using Component<ScalarT, IdxT>::yp_;
        using Component<ScalarT, IdxT>::J_;
        using Component<ScalarT, IdxT>::J_rows_buffer_;
        using Component<ScalarT, IdxT>::J_cols_buffer_;
        using Component<ScalarT, IdxT>::J_vals_buffer_;
        using Component<ScalarT, IdxT>::variable_indices_;
        using Component<ScalarT, IdxT>::residual_indices_;

      public:
        using RealT           = typename Component<ScalarT, IdxT>::RealT;
        using model_data_type = SignalDelayData<RealT, IdxT>;
        using signal_type     = SignalNode<ScalarT, IdxT>;

        SignalDelay();
        SignalDelay(signal_type* input, signal_type* output, const model_data_type& data);
        SignalDelay(const model_data_type& data);
        ~SignalDelay() = default;

        int  setGridKitComponentID(IdxT) override final;
        int  allocate() override final;
        int  verify() const override final;
        int  initialize() override final;
        int  tagDifferentiable() override final;
        int  evaluateResidual() override final;
        int  evaluateJacobian() override final;
        int  stepAccepted(RealT t) override final;
        void setMaxStepSize(RealT& hmax) const override final;

        /// Get the `ComponentSignals` from this `SignalDelay`
        auto getSignals()
            -> ComponentSignals<ScalarT,
                                IdxT,
                                SignalDelayInternalVariables,
                                SignalDelayExternalVariables>&
        {
          return signals_;
        }

      private:
        using HistoryT = History<RealT, RealT>;

        void    initializeParameters(const model_data_type& data);
        ScalarT delayedInput() const;
        RealT   heldValue(RealT lookup_time) const;
        RealT   linearValue(RealT lookup_time) const;

        template <typename ValueT>
        static RealT scalarValue(const ValueT& value);

      private:
        static constexpr size_t VALUE_INDEX = 0;

        RealT           delay_{0.0};
        RealT           initial_value_{0.0};
        RealT           max_step_size_{0.0};
        SignalDelayMode mode_{SignalDelayMode::LINEAR};

        HistoryT history_;

        /// Component signal extension
        ComponentSignals<ScalarT,
                         IdxT,
                         SignalDelayInternalVariables,
                         SignalDelayExternalVariables>
            signals_;
      };

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
