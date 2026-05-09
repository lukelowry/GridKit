/**
 * @file ConvolutionVecImpl.hpp
 * @brief Definition of a vector-valued vector-fitting convolution helper
 */

#pragma once

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVec.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVecData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      using Log = ::GridKit::Utilities::Logger;

      template <class ScalarT, typename IdxT>
      ConvolutionVec<ScalarT, IdxT>::ConvolutionVec()
      {
        size_ = 0;
      }

      template <class ScalarT, typename IdxT>
      ConvolutionVec<ScalarT, IdxT>::ConvolutionVec(const std::vector<signal_type*>& inputs,
                                                    const std::vector<signal_type*>& outputs,
                                                    const model_data_type&           data)
      {
        initializeParameters(data);
        attachInputSignals(inputs);
        assignOutputSignals(outputs);
      }

      template <class ScalarT, typename IdxT>
      ConvolutionVec<ScalarT, IdxT>::ConvolutionVec(const model_data_type& data)
      {
        initializeParameters(data);
      }

      template <class ScalarT, typename IdxT>
      void ConvolutionVec<ScalarT, IdxT>::initializeParameters(const model_data_type& data)
      {
        dimension_       = data.dimension;
        d_               = data.d;
        e_               = data.e;
        poles_           = data.p;
        input_couplings_ = data.b;
        output_residues_ = data.c;
        u0_              = data.u0;
        up0_             = data.up0;

        size_ = static_cast<IdxT>(static_cast<size_t>(2) * dimension_ + poles_.size());
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::attachInputSignals(const std::vector<signal_type*>& inputs)
      {
        input_signals_ = inputs;
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::assignOutputSignals(const std::vector<signal_type*>& outputs)
      {
        output_signals_ = outputs;
        linkOutputSignals();
        return 0;
      }

      template <class ScalarT, typename IdxT>
      void ConvolutionVec<ScalarT, IdxT>::linkOutputSignals()
      {
        const auto size = static_cast<size_t>(size_);
        if (output_signals_.size() != dimension_ || y_.size() != size || variable_indices_.size() != size)
        {
          return;
        }

        for (size_t i = 0; i < dimension_; ++i)
        {
          if (output_signals_[i] != nullptr)
          {
            const auto local_index = static_cast<IdxT>(zIndex(i));
            output_signals_[i]->set(&y_[zIndex(i)], &(this->getVariableIndex(local_index)));
          }
        }
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::setGridKitComponentID(IdxT component_id)
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::allocate()
      {
        const auto size = static_cast<size_t>(size_);
        f_.resize(size);
        y_.resize(size);
        yp_.resize(size);
        tag_.resize(size);
        variable_indices_.resize(size);
        residual_indices_.resize(size);

        ws_.resize(dimension_);
        for (auto& value : ws_)
        {
          value = 0.0;
        }

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        linkOutputSignals();

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::verify() const
      {
        int ret = 0;

        if (dimension_ == 0)
        {
          Log::error() << "ConvolutionVec: dimension must be positive\n";
          ret += 1;
        }

        const size_t matrix_size      = dimension_ * dimension_;
        const size_t mode_vector_size = poles_.size() * dimension_;

        if (d_.size() != matrix_size)
        {
          Log::error() << "ConvolutionVec: d matrix size must equal dimension^2\n";
          ret += 1;
        }
        if (e_.size() != matrix_size)
        {
          Log::error() << "ConvolutionVec: e matrix size must equal dimension^2\n";
          ret += 1;
        }
        if (input_couplings_.size() != mode_vector_size)
        {
          Log::error() << "ConvolutionVec: b vector size must equal pole_count * dimension\n";
          ret += 1;
        }
        if (output_residues_.size() != mode_vector_size)
        {
          Log::error() << "ConvolutionVec: c vector size must equal pole_count * dimension\n";
          ret += 1;
        }
        if (u0_.size() != dimension_)
        {
          Log::error() << "ConvolutionVec: u0 vector size must equal dimension\n";
          ret += 1;
        }
        if (up0_.size() != dimension_)
        {
          Log::error() << "ConvolutionVec: up0 vector size must equal dimension\n";
          ret += 1;
        }

        for (auto pole : poles_)
        {
          if (pole == 0.0)
          {
            Log::error() << "ConvolutionVec: poles must be nonzero\n";
            ret += 1;
          }
        }

        if (input_signals_.size() != dimension_)
        {
          Log::error() << "ConvolutionVec: input signal count must equal dimension\n";
          ret += 1;
        }
        for (size_t i = 0; i < input_signals_.size(); ++i)
        {
          if (input_signals_[i] == nullptr)
          {
            Log::error() << "ConvolutionVec: input signal node must not be null\n";
            ret += 1;
          }
          else if (!input_signals_[i]->linked())
          {
            Log::error() << "ConvolutionVec: input signal node attached with no linked source\n";
            ret += 1;
          }
        }

        if (output_signals_.size() != dimension_)
        {
          Log::error() << "ConvolutionVec: output signal count must equal dimension\n";
          ret += 1;
        }
        for (size_t i = 0; i < output_signals_.size(); ++i)
        {
          if (output_signals_[i] == nullptr)
          {
            Log::error() << "ConvolutionVec: output signal node must not be null\n";
            ret += 1;
          }
        }

        return ret;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::initialize()
      {
        for (size_t i = 0; i < dimension_; ++i)
        {
          y_[uIndex(i)]  = u0_[i];
          yp_[uIndex(i)] = up0_[i];
        }

        for (size_t k = 0; k < poles_.size(); ++k)
        {
          ScalarT input_value{0};
          ScalarT input_derivative{0};
          for (size_t i = 0; i < dimension_; ++i)
          {
            const auto coupling_index  = modeVectorIndex(k, i);
            input_value               += input_couplings_[coupling_index] * u0_[i];
            input_derivative          += input_couplings_[coupling_index] * up0_[i];
          }

          const auto state_index = xIndex(k);
          const auto pole        = poles_[k];

          y_[state_index]  = -input_value / pole - input_derivative / (pole * pole);
          yp_[state_index] = input_value + pole * y_[state_index];
        }

        for (size_t i = 0; i < dimension_; ++i)
        {
          ScalarT output{0};
          for (size_t j = 0; j < dimension_; ++j)
          {
            const auto coefficient_index  = matrixIndex(i, j);
            output                       += d_[coefficient_index] * y_[uIndex(j)];
            output                       += e_[coefficient_index] * yp_[uIndex(j)];
          }

          for (size_t k = 0; k < poles_.size(); ++k)
          {
            output += output_residues_[modeVectorIndex(k, i)] * y_[xIndex(k)];
          }

          y_[zIndex(i)]  = output;
          yp_[zIndex(i)] = 0.0;
        }

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::tagDifferentiable()
      {
        for (size_t i = 0; i < dimension_; ++i)
        {
          tag_[uIndex(i)] = true;
          tag_[zIndex(i)] = false;
        }

        for (size_t k = 0; k < poles_.size(); ++k)
        {
          tag_[xIndex(k)] = true;
        }

        return 0;
      }

      template <class ScalarT, typename IdxT>
      __attribute__((always_inline)) inline int ConvolutionVec<ScalarT, IdxT>::evaluateInternalResidual(
          ScalarT*                  y,
          ScalarT*                  yp,
          [[maybe_unused]] ScalarT* wb,
          ScalarT*                  ws,
          ScalarT*                  f)
      {
        for (size_t i = 0; i < dimension_; ++i)
        {
          f[uIndex(i)] = y[uIndex(i)] - ws[i];
        }

        for (size_t k = 0; k < poles_.size(); ++k)
        {
          ScalarT input_value{0};
          for (size_t i = 0; i < dimension_; ++i)
          {
            input_value += input_couplings_[modeVectorIndex(k, i)] * y[uIndex(i)];
          }

          f[xIndex(k)] = -yp[xIndex(k)] + input_value + poles_[k] * y[xIndex(k)];
        }

        for (size_t i = 0; i < dimension_; ++i)
        {
          ScalarT direct_sum{0};
          for (size_t j = 0; j < dimension_; ++j)
          {
            const auto coefficient_index  = matrixIndex(i, j);
            direct_sum                   += d_[coefficient_index] * y[uIndex(j)];
            direct_sum                   += e_[coefficient_index] * yp[uIndex(j)];
          }

          ScalarT residue_sum{0};
          for (size_t k = 0; k < poles_.size(); ++k)
          {
            residue_sum += output_residues_[modeVectorIndex(k, i)] * y[xIndex(k)];
          }

          f[zIndex(i)] = y[zIndex(i)] - direct_sum - residue_sum;
        }

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::evaluateResidual()
      {
        for (size_t i = 0; i < dimension_; ++i)
        {
          ws_[i] = 0.0;
          if (i < input_signals_.size() && input_signals_[i] != nullptr && input_signals_[i]->linked())
          {
            ws_[i] = input_signals_[i]->read();
          }
        }

        evaluateInternalResidual(y_.data(), yp_.data(), wb_.data(), ws_.data(), f_.data());

        return 0;
      }

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit
