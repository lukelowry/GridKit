#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VectorFitParameters
    {
      D,
      E,
      poles,
      residues
    };

    enum class VectorFitBuses : size_t
    {
      SIZE
    };

    enum class VectorFitSignalInputs : size_t
    {
      input_a,
      input_b,
      input_c,
      SIZE
    };

    enum class VectorFitSignalOutputs : size_t
    {
      out_a,
      out_b,
      out_c,
      SIZE
    };

    template <typename real_type, typename index_type>
    struct VectorFitData
      : public ComponentData<real_type,
                             index_type,
                             VectorFitParameters,
                             VectorFitBuses,
                             VectorFitSignalInputs,
                             VectorFitSignalOutputs,
                             std::monostate>
    {
      using Parameters           = VectorFitParameters;
      using Buses                = VectorFitBuses;
      using SignalInputs         = VectorFitSignalInputs;
      using SignalOutputs        = VectorFitSignalOutputs;
      using MonitorableVariables = std::monostate;
    };

    /**
     * @brief Constant and linear coefficients of a rational submodel.
     *
     * A consuming model that cannot yet realize rational dynamics reads these
     * coefficients directly and rejects a fit reporting `dynamic`.
     */
    template <typename real_type>
    struct RationalCoefficients
    {
      ABCMatrix<real_type> D{};            ///< Constant coefficient
      ABCMatrix<real_type> E{};            ///< Linear coefficient
      bool                 dynamic{false}; ///< True when the fit has poles
    };

    template <typename real_type, typename index_type>
    RationalCoefficients<real_type> rationalCoefficients(
        const VectorFitData<real_type, index_type>& data)
    {
      RationalCoefficients<real_type> result;
      if (data.parameters.contains(VectorFitParameters::D))
      {
        result.D = std::get<ABCMatrix<real_type>>(
            data.parameters.at(VectorFitParameters::D));
      }
      if (data.parameters.contains(VectorFitParameters::E))
      {
        result.E = std::get<ABCMatrix<real_type>>(
            data.parameters.at(VectorFitParameters::E));
      }
      if (data.parameters.contains(VectorFitParameters::poles))
      {
        result.dynamic = !std::get<std::vector<std::complex<real_type>>>(
                              data.parameters.at(VectorFitParameters::poles))
                              .empty();
      }
      return result;
    }
  } // namespace EMT
} // namespace GridKit
