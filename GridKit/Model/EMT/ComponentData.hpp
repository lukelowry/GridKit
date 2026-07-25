#pragma once

#include <array>
#include <complex>
#include <map>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <typename T>
    using ABCVector = std::array<T, 3>;

    template <typename T>
    using ABCMatrix = std::array<std::array<T, 3>, 3>;

    template <typename real_type, typename index_type>
    using ParameterValue = std::variant<bool,
                                        real_type,
                                        index_type,
                                        ABCVector<real_type>,
                                        ABCMatrix<real_type>,
                                        std::vector<std::complex<real_type>>,
                                        std::vector<ABCMatrix<std::complex<real_type>>>>;

    /**
     * @brief Common modeling data for fixed-ABC EMT components.
     */
    template <typename real_type,
              typename index_type,
              typename Parameters,
              typename Buses,
              typename SignalInputs,
              typename SignalOutputs,
              typename MonitorableVariables>
    struct ComponentData
    {
      std::string                                                 device_class;
      std::map<Parameters, ParameterValue<real_type, index_type>> parameters;
      std::map<Buses, index_type>                                 buses;
      std::map<SignalInputs, index_type>                          signal_inputs;
      std::map<SignalOutputs, index_type>                         signal_outputs;
      std::set<MonitorableVariables>                              monitored_variables;
      std::string                                                 disambiguation_string;
    };
  } // namespace EMT
} // namespace GridKit
