/**
 * @file ConvolutionVFImpl.hpp
 * @brief Definition of a vector-fitting convolution helper
 */

#pragma once

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVF.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVFData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      using Log = ::GridKit::Utilities::Logger;

      /**
       * @brief Constructs a vector-fitting convolution helper without parameters
       */
      template <class ScalarT, typename IdxT>
      ConvolutionVF<ScalarT, IdxT>::ConvolutionVF()
      {
        size_ = static_cast<IdxT>(X_OFFSET);
      }

      /**
       * @brief Constructs a vector-fitting convolution helper from signals and data
       */
      template <class ScalarT, typename IdxT>
      ConvolutionVF<ScalarT, IdxT>::ConvolutionVF(signal_type*           input,
                                                  signal_type*           output,
                                                  const model_data_type& data)
      {
        signals_.template attachSignalNode<ConvolutionVFExternalVariables::U>(input);
        signals_.template assignSignalNode<ConvolutionVFInternalVariables::Z>(output);
        initializeParameters(data);
      }

      /**
       * @brief Constructs a vector-fitting convolution helper from data
       */
      template <class ScalarT, typename IdxT>
      ConvolutionVF<ScalarT, IdxT>::ConvolutionVF(const model_data_type& data)
      {
        initializeParameters(data);
      }

      /**
       * @brief Helper function to extract and assign model parameters
       */
      template <class ScalarT, typename IdxT>
      void ConvolutionVF<ScalarT, IdxT>::initializeParameters(const model_data_type& data)
      {
        d_        = data.d;
        e_        = data.e;
        u0_       = data.u0;
        up0_      = data.up0;
        poles_    = data.p;
        residues_ = data.r;

        size_ = static_cast<IdxT>(X_OFFSET + poles_.size());
      }

      /**
       * @brief Set the component ID
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      /**
       * @brief Allocate memory for model
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::allocate()
      {
        auto size = static_cast<size_t>(size_);
        f_.resize(size);
        y_.resize(size);
        yp_.resize(size);
        tag_.resize(size);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        ws_.resize(1);
        ws_indices_.resize(1);
        ws_[0]         = 0.0;
        ws_indices_[0] = INVALID_INDEX<IdxT>;

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        if (signals_.template isAssigned<ConvolutionVFInternalVariables::Z>())
        {
          signals_.template getSignalNode<ConvolutionVFInternalVariables::Z>()->set(
              &y_[Z_INDEX], &(this->getVariableIndex(static_cast<IdxT>(Z_INDEX))));
        }

        return 0;
      }

      /**
       * @brief Verify coefficient and signal configuration
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::verify() const
      {
        int ret = 0;

        if (poles_.size() != residues_.size())
        {
          Log::error() << "ConvolutionVF: pole and residue vector sizes must match\n";
          ret += 1;
        }

        for (auto pole : poles_)
        {
          if (pole == 0.0)
          {
            Log::error() << "ConvolutionVF: poles must be nonzero\n";
            ret += 1;
          }
        }

        if (signals_.template isAttached<ConvolutionVFExternalVariables::U>())
        {
          if (!signals_.template isLinked<ConvolutionVFExternalVariables::U>())
          {
            Log::error() << "ConvolutionVF: input signal U attached with no linked source\n";
            ret += 1;
          }
        }
        else
        {
          Log::error() << "ConvolutionVF: required input signal U is not attached\n";
          ret += 1;
        }

        if (!signals_.template isAssigned<ConvolutionVFInternalVariables::Z>())
        {
          Log::error() << "ConvolutionVF: required output signal Z is not assigned\n";
          ret += 1;
        }

        return ret;
      }

      /**
       * @brief Initialization of the convolution helper
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::initialize()
      {
        y_[U_INDEX]  = u0_;
        yp_[U_INDEX] = up0_;

        ScalarT residue_sum{0};
        for (size_t n = 0; n < poles_.size(); ++n)
        {
          const auto state_index = X_OFFSET + n;

          y_[state_index]   = -u0_ / poles_[n] - up0_ / (poles_[n] * poles_[n]);
          yp_[state_index]  = u0_ + poles_[n] * y_[state_index];
          residue_sum      += residues_[n] * y_[state_index];
        }

        y_[Z_INDEX]  = d_ * y_[U_INDEX] + e_ * yp_[U_INDEX] + residue_sum;
        yp_[Z_INDEX] = 0.0;

        return 0;
      }

      /**
       * @brief Identify differential variables
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::tagDifferentiable()
      {
        tag_[U_INDEX] = true;
        tag_[Z_INDEX] = false;

        for (IdxT i = static_cast<IdxT>(X_OFFSET); i < size_; ++i)
        {
          tag_[static_cast<size_t>(i)] = true;
        }

        return 0;
      }

      /**
       * @brief Internal residuals
       */
      template <class ScalarT, typename IdxT>
      __attribute__((always_inline)) inline int ConvolutionVF<ScalarT, IdxT>::evaluateInternalResidual(
          ScalarT*                  y,
          ScalarT*                  yp,
          [[maybe_unused]] ScalarT* wb,
          ScalarT*                  ws,
          ScalarT*                  f)
      {
        ScalarT u     = y[U_INDEX];
        ScalarT z     = y[Z_INDEX];
        ScalarT up    = yp[U_INDEX];
        ScalarT input = ws[0];

        f[U_INDEX] = u - input;

        ScalarT residue_sum{0};
        for (size_t n = 0; n < poles_.size(); ++n)
        {
          const auto state_index = X_OFFSET + n;
          ScalarT    x           = y[state_index];
          ScalarT    xp          = yp[state_index];

          residue_sum    += residues_[n] * x;
          f[state_index]  = -xp + u + poles_[n] * x;
        }

        f[Z_INDEX] = z - d_ * u - e_ * up - residue_sum;

        return 0;
      }

      /**
       * @brief Residuals of system equations
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::evaluateResidual()
      {
        if (signals_.template isAttached<ConvolutionVFExternalVariables::U>())
        {
          ws_[0]         = signals_.template readExternalVariable<ConvolutionVFExternalVariables::U>();
          ws_indices_[0] = signals_.template readExternalVariableIndex<ConvolutionVFExternalVariables::U>();
        }

        evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), f_.data());

        return 0;
      }

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
