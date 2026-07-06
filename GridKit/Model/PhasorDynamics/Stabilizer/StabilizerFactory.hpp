/**
 * @file StabilizerFactory.hpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Factory for constructing stabilizer models from modeling data.
 */

#pragma once

#include <cstddef>
#include <variant>

#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/Ieeest.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/IeeestData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Stabilizer
    {
      /**
       * @brief Creates stabilizer components of the concrete order implied by
       *        their modeling data.
       *
       * The notch-filter order of a `Ieeest` is a template parameter, while
       * modeling data determines the order at runtime. This factory resolves
       * the runtime decision to the matching `Ieeest` instantiation and wires
       * the input/output signal nodes.
       */
      template <typename scalar_type, typename index_type>
      class StabilizerFactory
      {
      public:
        using ScalarT     = scalar_type;
        using IdxT        = index_type;
        using RealT       = typename Component<ScalarT, IdxT>::RealT;
        using ComponentT  = Component<ScalarT, IdxT>;
        using SignalT     = SignalNode<ScalarT, IdxT>;
        using IeeestDataT = IeeestData<RealT, IdxT>;

        StabilizerFactory() = delete;

        /**
         * @brief Create the `Ieeest` of the order implied by `data`.
         *
         * @param[in] data   IEEEST modeling data; the notch-filter order is
         *                   derived from its A1..A4 parameters.
         * @param[in] input  Stabilizer input signal node; may be `nullptr`.
         * @param[in] output Stabilizer output signal node; may be `nullptr`.
         * @return Newly allocated stabilizer; the caller assumes ownership.
         */
        static ComponentT* create(const IeeestDataT& data, SignalT* input, SignalT* output)
        {
          using Params = typename IeeestDataT::Parameters;

          const RealT A1 = readParameter(data, Params::A1);
          const RealT A2 = readParameter(data, Params::A2);
          const RealT A3 = readParameter(data, Params::A3);
          const RealT A4 = readParameter(data, Params::A4);

          const auto a = notchCoefficients(A1, A2, A3, A4);

          switch (notchOrder(a[0], a[1], a[2], a[3]))
          {
          case 0:
            return createIeeest<0>(data, input, output);
          case 1:
            return createIeeest<1>(data, input, output);
          case 2:
            return createIeeest<2>(data, input, output);
          case 3:
            return createIeeest<3>(data, input, output);
          default:
            // notchOrder yields at most 4
            return createIeeest<4>(data, input, output);
          }
        }

      private:
        /// Create a `Ieeest` of compile-time order `order` and wire its signal nodes
        template <size_t order>
        static ComponentT* createIeeest(const IeeestDataT& data, SignalT* input, SignalT* output)
        {
          auto* stabilizer = new Ieeest<ScalarT, IdxT, order>(data);

          if (input != nullptr)
          {
            stabilizer->getSignals().template attachSignalNode<IeeestExternalVariables::U>(input);
          }

          if (output != nullptr)
          {
            constexpr auto VSS = IeeestInternalVariables<order>::VSS;
            stabilizer->getSignals().template assignSignalNode<VSS>(output);
          }

          return stabilizer;
        }

        /// Read a real-valued parameter from `data`, defaulting to zero
        static RealT readParameter(const IeeestDataT& data, typename IeeestDataT::Parameters key)
        {
          if (!data.parameters.contains(key))
          {
            return ZERO<RealT>;
          }

          const auto& value = data.parameters.at(key);
          if (const auto* real_value = std::get_if<RealT>(&value))
          {
            return *real_value;
          }
          if (const auto* index_value = std::get_if<IdxT>(&value))
          {
            return static_cast<RealT>(*index_value);
          }

          return ZERO<RealT>;
        }
      };

    } // namespace Stabilizer
  } // namespace PhasorDynamics
} // namespace GridKit
