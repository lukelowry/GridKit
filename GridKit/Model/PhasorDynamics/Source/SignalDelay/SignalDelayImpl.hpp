/**
 * @file SignalDelayImpl.hpp
 * @brief Definition of a signal delay
 */

#pragma once

#include <type_traits>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/Source/SignalDelay/SignalDelay.hpp>
#include <GridKit/Model/PhasorDynamics/Source/SignalDelay/SignalDelayData.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      using Log = ::GridKit::Utilities::Logger;

      /**
       * @brief Constructs a signal delay without parameters
       */
      template <class ScalarT, typename IdxT>
      SignalDelay<ScalarT, IdxT>::SignalDelay()
      {
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
      }

      /**
       * @brief Constructs a signal delay from signals and data
       */
      template <class ScalarT, typename IdxT>
      SignalDelay<ScalarT, IdxT>::SignalDelay(signal_type*           input,
                                              signal_type*           output,
                                              const model_data_type& data)
      {
        signals_.template attachSignalNode<SignalDelayExternalVariables::INPUT>(input);
        signals_.template assignSignalNode<SignalDelayInternalVariables::VALUE>(output);
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
        initializeParameters(data);
      }

      /**
       * @brief Constructs a signal delay from data
       */
      template <class ScalarT, typename IdxT>
      SignalDelay<ScalarT, IdxT>::SignalDelay(const model_data_type& data)
      {
        size_  = 1;
        time_  = 0.0;
        alpha_ = 0.0;
        initializeParameters(data);
      }

      /**
       * @brief Helper function to extract and assign model parameters
       */
      template <class ScalarT, typename IdxT>
      void SignalDelay<ScalarT, IdxT>::initializeParameters(const model_data_type& data)
      {
        delay_         = data.delay;
        initial_value_ = data.initial_value;
        max_step_size_ = data.max_step_size;
        mode_          = data.mode;
        history_.configure(delay_, max_step_size_);
      }

      /**
       * @brief Set the component ID
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      /**
       * @brief Allocate memory for model
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::allocate()
      {
        auto size = static_cast<size_t>(size_);
        f_.resize(size);
        y_.resize(size);
        yp_.resize(size);
        tag_.resize(size);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        this->setVariableIndex(static_cast<IdxT>(VALUE_INDEX), static_cast<IdxT>(VALUE_INDEX));
        this->setResidualIndex(static_cast<IdxT>(VALUE_INDEX), static_cast<IdxT>(VALUE_INDEX));

        if (signals_.template isAssigned<SignalDelayInternalVariables::VALUE>())
        {
          signals_.template getSignalNode<SignalDelayInternalVariables::VALUE>()->set(
              &y_[VALUE_INDEX], &(this->getVariableIndex(static_cast<IdxT>(VALUE_INDEX))));
        }

        return 0;
      }

      /**
       * @brief Verify delay parameters and signal configuration
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::verify() const
      {
        int ret = 0;

        if (delay_ < 0.0)
        {
          Log::error() << "SignalDelay: delay must be nonnegative\n";
          ret += 1;
        }

        if (signals_.template isAttached<SignalDelayExternalVariables::INPUT>())
        {
          if (!signals_.template isLinked<SignalDelayExternalVariables::INPUT>())
          {
            Log::error() << "SignalDelay: input signal INPUT attached with no linked source\n";
            ret += 1;
          }
        }
        else
        {
          Log::error() << "SignalDelay: required input signal INPUT is not attached\n";
          ret += 1;
        }

        if (!signals_.template isAssigned<SignalDelayInternalVariables::VALUE>())
        {
          Log::error() << "SignalDelay: required output signal VALUE is not assigned\n";
          ret += 1;
        }

        return ret;
      }

      /**
       * @brief Initialization of the signal delay
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::initialize()
      {
        history_.configure(delay_, max_step_size_);
        history_.clear();

        if (delay_ == 0.0 && signals_.template isAttached<SignalDelayExternalVariables::INPUT>()
            && signals_.template isLinked<SignalDelayExternalVariables::INPUT>())
        {
          y_[VALUE_INDEX] = signals_.template readExternalVariable<SignalDelayExternalVariables::INPUT>();
        }
        else
        {
          y_[VALUE_INDEX] = initial_value_;
        }

        yp_[VALUE_INDEX] = 0.0;
        return 0;
      }

      /**
       * @brief Identify differential variables
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::tagDifferentiable()
      {
        tag_[VALUE_INDEX] = false;
        return 0;
      }

      /**
       * @brief Residuals of system equations
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::evaluateResidual()
      {
        f_[VALUE_INDEX] = y_[VALUE_INDEX] - delayedInput();
        return 0;
      }

      /**
       * @brief Record input signal value after an accepted solver step
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::stepAccepted(RealT t)
      {
        if (delay_ == 0.0)
        {
          history_.clear();
          return 0;
        }

        const auto input = signals_.template readExternalVariable<SignalDelayExternalVariables::INPUT>();
        const auto value = scalarValue(input);

        history_.record(t, value);
        return 0;
      }

      /**
       * @brief Request a maximum accepted solver step for delay history
       */
      template <class ScalarT, typename IdxT>
      void SignalDelay<ScalarT, IdxT>::setMaxStepSize(RealT& hmax) const
      {
        history_.setMaxStepSize(hmax);
      }

      /**
       * @brief Delayed signal value for the current residual evaluation
       */
      template <class ScalarT, typename IdxT>
      ScalarT SignalDelay<ScalarT, IdxT>::delayedInput() const
      {
        if (delay_ == 0.0)
        {
          return signals_.template readExternalVariable<SignalDelayExternalVariables::INPUT>();
        }

        const auto lookup_time = time_ - delay_;
        if (mode_ == SignalDelayMode::HOLD)
        {
          return ScalarT{heldValue(lookup_time)};
        }

        return ScalarT{linearValue(lookup_time)};
      }

      /**
       * @brief Zero-order hold lookup in accepted-step signal history
       */
      template <class ScalarT, typename IdxT>
      typename SignalDelay<ScalarT, IdxT>::RealT SignalDelay<ScalarT, IdxT>::heldValue(RealT lookup_time) const
      {
        return history_.heldValue(lookup_time, initial_value_);
      }

      /**
       * @brief Linear interpolation lookup in accepted-step signal history
       */
      template <class ScalarT, typename IdxT>
      typename SignalDelay<ScalarT, IdxT>::RealT SignalDelay<ScalarT, IdxT>::linearValue(RealT lookup_time) const
      {
        auto interpolate = [](RealT lower, RealT upper, RealT theta)
        {
          return lower + theta * (upper - lower);
        };

        return history_.interpolatedValue(lookup_time, initial_value_, interpolate);
      }

      /**
       * @brief Extract a real value from the scalar type
       */
      template <class ScalarT, typename IdxT>
      template <typename ValueT>
      typename SignalDelay<ScalarT, IdxT>::RealT SignalDelay<ScalarT, IdxT>::scalarValue(const ValueT& value)
      {
        if constexpr (std::is_arithmetic_v<ValueT>)
        {
          return static_cast<RealT>(value);
        }
        else
        {
          return static_cast<RealT>(value.getValue());
        }
      }

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
