/**
 * @file SignalDelayData.hpp
 * @brief Modeling data for a signal delay
 */

#pragma once

#include <GridKit/Model/PhasorDynamics/ComponentData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /// Signal delay history lookup mode
      enum class SignalDelayMode
      {
        LINEAR, ///< Piecewise-linear interpolation between accepted-step samples
        HOLD    ///< Zero-order hold of the last accepted-step sample
      };

      /// Scalar parameters for a signal delay
      enum class SignalDelayParameters
      {
        delay,
        initial_value,
        max_step_size
      };

      /// Signal ports for a signal delay
      enum class SignalDelayPorts
      {
        input,
        output
      };

      /// Placeholder enum for monitor compatibility
      enum class SignalDelayMonitorableVariables
      {
        NONE
      };

      /**
       * @brief Contains modeling data for a signal delay
       */
      template <typename RealT, typename IdxT>
      struct SignalDelayData : public ComponentData<RealT,
                                                    IdxT,
                                                    SignalDelayParameters,
                                                    SignalDelayPorts,
                                                    SignalDelayMonitorableVariables>
      {
        SignalDelayData() = default;

        using Parameters           = SignalDelayParameters;
        using Ports                = SignalDelayPorts;
        using MonitorableVariables = SignalDelayMonitorableVariables;

        RealT           delay{0.0};                    ///< Fixed signal delay
        RealT           initial_value{0.0};            ///< Value before accepted-step history exists
        RealT           max_step_size{0.0};            ///< Maximum accepted solver step; nonpositive uses delay
        SignalDelayMode mode{SignalDelayMode::LINEAR}; ///< History lookup mode
      };

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
