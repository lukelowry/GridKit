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

        complex_pole_real_            = data.complex_p_real;
        complex_pole_imag_            = data.complex_p_imag;
        complex_input_couplings_real_ = data.complex_b_real;
        complex_input_couplings_imag_ = data.complex_b_imag;
        complex_output_residues_real_ = data.complex_c_real;
        complex_output_residues_imag_ = data.complex_c_imag;

        u0_  = data.u0;
        up0_ = data.up0;

        complex_state_offset_ = static_cast<size_t>(2) * dimension_ + poles_.size();
        size_                 = static_cast<IdxT>(static_cast<size_t>(2) * dimension_ + memoryStateCount());
      }

      template <class ScalarT, typename IdxT>
      auto ConvolutionVec<ScalarT, IdxT>::jacobianEntryCapacity() const -> size_t
      {
        const auto real_mode_count    = realModeCount();
        const auto complex_pair_count = complexPairCount();

        return static_cast<size_t>(3) * dimension_
               + dimension_ * dimension_
               + static_cast<size_t>(2) * dimension_ * real_mode_count
               + real_mode_count
               + complex_pair_count * (static_cast<size_t>(4) * dimension_ + static_cast<size_t>(4));
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
        int ret = 0;

        if (dimension_ == 0)
        {
          Log::error() << "ConvolutionVec: dimension must be positive\n";
          ret += 1;
        }

        const size_t matrix_size              = dimension_ * dimension_;
        const size_t real_mode_vector_size    = poles_.size() * dimension_;
        const size_t complex_pair_count       = complex_pole_real_.size();
        const size_t complex_mode_vector_size = complex_pair_count * dimension_;

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
        if (input_couplings_.size() != real_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: b vector size must equal pole_count * dimension\n";
          ret += 1;
        }
        if (output_residues_.size() != real_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: c vector size must equal pole_count * dimension\n";
          ret += 1;
        }
        if (complex_pole_imag_.size() != complex_pair_count)
        {
          Log::error() << "ConvolutionVec: complex pole real and imaginary vector sizes must match\n";
          ret += 1;
        }
        if (complex_input_couplings_real_.size() != complex_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: complex_b_real size must equal complex_pair_count * dimension\n";
          ret += 1;
        }
        if (complex_input_couplings_imag_.size() != complex_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: complex_b_imag size must equal complex_pair_count * dimension\n";
          ret += 1;
        }
        if (complex_output_residues_real_.size() != complex_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: complex_c_real size must equal complex_pair_count * dimension\n";
          ret += 1;
        }
        if (complex_output_residues_imag_.size() != complex_mode_vector_size)
        {
          Log::error() << "ConvolutionVec: complex_c_imag size must equal complex_pair_count * dimension\n";
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
        for (size_t pair = 0; pair < complex_pole_imag_.size(); ++pair)
        {
          if (complex_pole_imag_[pair] <= 0.0)
          {
            Log::error() << "ConvolutionVec: complex pole imaginary parts must be positive\n";
            ret += 1;
          }
          if (pair < complex_pole_real_.size()
              && complex_pole_real_[pair] == 0.0
              && complex_pole_imag_[pair] == 0.0)
          {
            Log::error() << "ConvolutionVec: complex poles must be nonzero\n";
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

        for (size_t pair = 0; pair < complexPairCount(); ++pair)
        {
          ScalarT input_value_real{0};
          ScalarT input_value_imag{0};
          ScalarT input_derivative_real{0};
          ScalarT input_derivative_imag{0};
          for (size_t i = 0; i < dimension_; ++i)
          {
            const auto coupling_index  = modeVectorIndex(pair, i);
            input_value_real          += complex_input_couplings_real_[coupling_index] * u0_[i];
            input_value_imag          += complex_input_couplings_imag_[coupling_index] * u0_[i];
            input_derivative_real     += complex_input_couplings_real_[coupling_index] * up0_[i];
            input_derivative_imag     += complex_input_couplings_imag_[coupling_index] * up0_[i];
          }

          const auto real_state_index = complexRealStateIndex(pair);
          const auto imag_state_index = complexImagStateIndex(pair);
          const auto pole_real        = complex_pole_real_[pair];
          const auto pole_imag        = complex_pole_imag_[pair];
          const auto pole_norm_sq     = pole_real * pole_real + pole_imag * pole_imag;
          const auto inv_pole_real    = pole_real / pole_norm_sq;
          const auto inv_pole_imag    = -pole_imag / pole_norm_sq;
          const auto inv_pole_sq_real = inv_pole_real * inv_pole_real - inv_pole_imag * inv_pole_imag;
          const auto inv_pole_sq_imag = static_cast<RealT>(2.0) * inv_pole_real * inv_pole_imag;

          y_[real_state_index] = -(input_value_real * inv_pole_real - input_value_imag * inv_pole_imag)
                                 - (input_derivative_real * inv_pole_sq_real
                                    - input_derivative_imag * inv_pole_sq_imag);
          y_[imag_state_index] = -(input_value_real * inv_pole_imag + input_value_imag * inv_pole_real)
                                 - (input_derivative_real * inv_pole_sq_imag
                                    + input_derivative_imag * inv_pole_sq_real);

          yp_[real_state_index] = input_value_real
                                  + pole_real * y_[real_state_index]
                                  - pole_imag * y_[imag_state_index];
          yp_[imag_state_index] = input_value_imag
                                  + pole_imag * y_[real_state_index]
                                  + pole_real * y_[imag_state_index];
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
          for (size_t pair = 0; pair < complexPairCount(); ++pair)
          {
            const auto residue_index  = modeVectorIndex(pair, i);
            output                   += static_cast<RealT>(2.0)
                      * (complex_output_residues_real_[residue_index] * y_[complexRealStateIndex(pair)]
                         - complex_output_residues_imag_[residue_index] * y_[complexImagStateIndex(pair)]);
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
        for (size_t pair = 0; pair < complexPairCount(); ++pair)
        {
          tag_[complexRealStateIndex(pair)] = true;
          tag_[complexImagStateIndex(pair)] = true;
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

        const auto real_mode_count    = realModeCount();
        const auto complex_pair_count = complexPairCount();

        for (size_t k = 0; k < real_mode_count; ++k)
        {
          ScalarT input_value{0};
          for (size_t i = 0; i < dimension_; ++i)
          {
            input_value += input_couplings_[modeVectorIndex(k, i)] * y[uIndex(i)];
          }

          f[xIndex(k)] = -yp[xIndex(k)] + input_value + poles_[k] * y[xIndex(k)];
        }

        for (size_t pair = 0; pair < complex_pair_count; ++pair)
        {
          ScalarT input_value_real{0};
          ScalarT input_value_imag{0};
          for (size_t i = 0; i < dimension_; ++i)
          {
            const auto coupling_index  = modeVectorIndex(pair, i);
            input_value_real          += complex_input_couplings_real_[coupling_index] * y[uIndex(i)];
            input_value_imag          += complex_input_couplings_imag_[coupling_index] * y[uIndex(i)];
          }

          const auto real_state_index = complexRealStateIndex(pair);
          const auto imag_state_index = complexImagStateIndex(pair);
          const auto pole_real        = complex_pole_real_[pair];
          const auto pole_imag        = complex_pole_imag_[pair];

          f[real_state_index] = -yp[real_state_index]
                                + input_value_real
                                + pole_real * y[real_state_index]
                                - pole_imag * y[imag_state_index];
          f[imag_state_index] = -yp[imag_state_index]
                                + input_value_imag
                                + pole_imag * y[real_state_index]
                                + pole_real * y[imag_state_index];
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
          for (size_t k = 0; k < real_mode_count; ++k)
          {
            residue_sum += output_residues_[modeVectorIndex(k, i)] * y[xIndex(k)];
          }
          for (size_t pair = 0; pair < complex_pair_count; ++pair)
          {
            const auto residue_index  = modeVectorIndex(pair, i);
            residue_sum              += static_cast<RealT>(2.0)
                           * (complex_output_residues_real_[residue_index] * y[complexRealStateIndex(pair)]
                              - complex_output_residues_imag_[residue_index] * y[complexImagStateIndex(pair)]);
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
