#pragma once

#include <array>
#include <complex>
#include <map>
#include <set>
#include <string>
#include <type_traits>
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
                                        ABCVector<index_type>,
                                        ABCVector<real_type>,
                                        ABCMatrix<real_type>,
                                        std::vector<std::complex<real_type>>,
                                        std::vector<ABCMatrix<std::complex<real_type>>>>;

    /**
     * @brief Unified interface for fixed-ABC EMT `Component` data containers
     *
     * @tparam real_type  Real parameter data type
     * @tparam index_type Integer parameter data type
     */
    template <typename real_type,
              typename index_type,
              typename Parameters,
              typename Buses,
              typename SignalInputs,
              typename SignalOutputs,
              typename MonitorableVariables>
      requires std::is_enum_v<Parameters>
               && std::is_enum_v<Buses>
               && std::is_enum_v<SignalInputs>
               && std::is_enum_v<SignalOutputs>
    struct ComponentData
    {
      /// Real value type
      using RealT = real_type;
      /// Index type
      using IdxT  = index_type;

      /// Class of device this is for
      std::string device_class;

      /// Mapping of parameters to parameter values
      std::map<Parameters, ParameterValue<RealT, IdxT>> parameters;

      /// Mapping of terminal attachments to bus identifiers
      std::map<Buses, IdxT> buses;

      /// Mapping of signal inputs to signal identifiers
      std::map<SignalInputs, IdxT> signal_inputs;

      /// Mapping of signal outputs to signal identifiers
      std::map<SignalOutputs, IdxT> signal_outputs;

      /// Set of variables being monitored
      std::set<MonitorableVariables> monitored_variables;

      std::string disambiguation_string; ///< Disambiguation string for this device

    protected:
      ComponentData() = default;
    };
  } // namespace EMT
} // namespace GridKit
