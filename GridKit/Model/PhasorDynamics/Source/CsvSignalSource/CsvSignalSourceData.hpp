/**
 * @file CsvSignalSourceData.hpp
 * @brief Modeling data for a CSV-backed signal source
 */

#pragma once

#include <filesystem>
#include <string>

#include <GridKit/Model/PhasorDynamics/ComponentData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /// Scalar parameters for a CSV-backed signal source
      enum class CsvSignalSourceParameters
      {
        value_scale,
        value_offset
      };

      /// Signal ports for a CSV-backed signal source
      enum class CsvSignalSourcePorts
      {
        output
      };

      /// Placeholder enum for monitor compatibility
      enum class CsvSignalSourceMonitorableVariables
      {
        NONE
      };

      /**
       * @brief Contains modeling data for a CSV-backed signal source.
       *
       * The file path and column names are stored as typed fields because
       * `ComponentData::parameters` currently carries scalar values only.
       */
      template <typename RealT, typename IdxT>
      struct CsvSignalSourceData : public ComponentData<RealT,
                                                        IdxT,
                                                        CsvSignalSourceParameters,
                                                        CsvSignalSourcePorts,
                                                        CsvSignalSourceMonitorableVariables>
      {
        CsvSignalSourceData() = default;

        using Parameters           = CsvSignalSourceParameters;
        using Ports                = CsvSignalSourcePorts;
        using MonitorableVariables = CsvSignalSourceMonitorableVariables;

        std::filesystem::path file;              ///< CSV file containing input samples
        std::string           time_column{"t"};  ///< Time column name
        std::string           value_column{"u"}; ///< Value column name
        RealT                 value_scale{1.0};  ///< Multiplicative value scale
        RealT                 value_offset{0.0}; ///< Additive value offset
      };

    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit
