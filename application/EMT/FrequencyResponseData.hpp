/**
 * @file FrequencyResponseData.hpp
 *
 * @brief Input data for the EMT FrequencyResponse application.
 *
 */

#pragma once

#include <filesystem>
#include <set>
#include <string>

#include <GridKit/Model/EMT/Parameters/OverheadData.hpp>
#include <GridKit/Solver/Dynamic/IdaOptions.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Application
    {
      struct FrequencyGrid
      {
        double      start{0.0}; ///< Start frequency in Hz.
        double      stop{0.0};  ///< Stop frequency in Hz.
        size_t      points{0};
        std::string scale{"log"};
      };

      struct FrequencyResponseData
      {
        using MonitorVariable =
            Parameters::OverheadData<double, size_t>::MonitorableVariables;

        std::filesystem::path                 model;
        FrequencyGrid                         frequency;
        AnalysisManager::Sundials::IdaOptions ida;
        std::filesystem::path                 output_file;
        std::set<MonitorVariable>             variables;
      };
    } // namespace Application
  } // namespace EMT
} // namespace GridKit
