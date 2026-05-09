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
        GridKit::EMT::RationalApproxData<RealT, IdxT> approx_data;
        approx_data.dimension = data.dimension;
        approx_data.d         = data.d;
        approx_data.e         = data.e;

        approx_data.p = data.p;
        approx_data.b = data.b;
        approx_data.c = data.c;

        approx_data.complex_p_real = data.complex_p_real;
        approx_data.complex_p_imag = data.complex_p_imag;
        approx_data.complex_b_real = data.complex_b_real;
        approx_data.complex_b_imag = data.complex_b_imag;
        approx_data.complex_c_real = data.complex_c_real;
        approx_data.complex_c_imag = data.complex_c_imag;

        dimension_ = data.dimension;
        approximation_.setData(approx_data);

        u0_  = data.u0;
        up0_ = data.up0;

        size_ = static_cast<IdxT>(static_cast<size_t>(2) * dimension_ + memoryStateCount());
      }

      template <class ScalarT, typename IdxT>
      auto ConvolutionVec<ScalarT, IdxT>::jacobianEntryCapacity() const -> size_t
      {
        return static_cast<size_t>(2) * dimension_
               + dimension_
               + approximation_.outputJacobianEntryCount()
               + approximation_.stateJacobianEntryCount();
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
        output_.resize(dimension_);

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        const auto max_nnz = jacobianEntryCapacity();
        if (J_rows_buffer_ == nullptr && max_nnz > 0)
        {
          J_rows_buffer_ = new IdxT[max_nnz];
          J_cols_buffer_ = new IdxT[max_nnz];
          J_vals_buffer_ = new RealT[max_nnz];
          J_.reserve(static_cast<IdxT>(max_nnz));
        }

        linkOutputSignals();

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::verify() const
      {
        int ret = approximation_.verify();

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

        if (memoryStateCount() > 0)
        {
          approximation_.initialize(y_.data() + uIndex(0),
                                    yp_.data() + uIndex(0),
                                    y_.data() + memoryStateIndex(0),
                                    yp_.data() + memoryStateIndex(0));
        }

        approximation_.evaluateOutput(y_.data() + uIndex(0),
                                      yp_.data() + uIndex(0),
                                      y_.data() + memoryStateIndex(0),
                                      y_.data() + zIndex(0));
        for (size_t i = 0; i < dimension_; ++i)
        {
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

        for (size_t state = 0; state < memoryStateCount(); ++state)
        {
          tag_[memoryStateIndex(state)] = true;
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

        if (memoryStateCount() > 0)
        {
          approximation_.evaluateStateResidual(y + uIndex(0),
                                               y + memoryStateIndex(0),
                                               yp + memoryStateIndex(0),
                                               f + memoryStateIndex(0));
        }

        approximation_.evaluateOutput(y + uIndex(0),
                                      yp + uIndex(0),
                                      y + memoryStateIndex(0),
                                      output_.data());

        for (size_t i = 0; i < dimension_; ++i)
        {
          f[zIndex(i)] = y[zIndex(i)] - output_[i];
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
