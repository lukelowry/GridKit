/**
 * @file RationalApproxImpl.hpp
 * @brief Implementation of real-valued rational matrix approximation blocks.
 */

#pragma once

#include <GridKit/Model/EMT/RationalApprox/RationalApprox.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using Log = ::GridKit::Utilities::Logger;

    template <class ScalarT, typename IdxT>
    RationalApprox<ScalarT, IdxT>::RationalApprox(const DataT& data)
    {
      setData(data);
    }

    template <class ScalarT, typename IdxT>
    void RationalApprox<ScalarT, IdxT>::setData(const DataT& data)
    {
      dimension_ = data.dimension;

      d_ = data.d;
      e_ = data.e;

      poles_           = data.p;
      input_couplings_ = data.b;
      output_residues_ = data.c;

      complex_pole_real_            = data.complex_p_real;
      complex_pole_imag_            = data.complex_p_imag;
      complex_input_couplings_real_ = data.complex_b_real;
      complex_input_couplings_imag_ = data.complex_b_imag;
      complex_output_residues_real_ = data.complex_c_real;
      complex_output_residues_imag_ = data.complex_c_imag;
    }

    template <class ScalarT, typename IdxT>
    int RationalApprox<ScalarT, IdxT>::verify() const
    {
      int ret = 0;

      if (dimension_ == 0)
      {
        Log::error() << "RationalApprox: dimension must be positive\n";
        ret += 1;
      }

      const size_t matrix_size              = dimension_ * dimension_;
      const size_t real_mode_vector_size    = realPoleCount() * dimension_;
      const size_t complex_pair_count       = complexPairCount();
      const size_t complex_mode_vector_size = complex_pair_count * dimension_;

      if (d_.size() != matrix_size)
      {
        Log::error() << "RationalApprox: d matrix size must equal dimension^2\n";
        ret += 1;
      }
      if (e_.size() != matrix_size)
      {
        Log::error() << "RationalApprox: e matrix size must equal dimension^2\n";
        ret += 1;
      }
      if (input_couplings_.size() != real_mode_vector_size)
      {
        Log::error() << "RationalApprox: b vector size must equal real pole count * dimension\n";
        ret += 1;
      }
      if (output_residues_.size() != real_mode_vector_size)
      {
        Log::error() << "RationalApprox: c vector size must equal real pole count * dimension\n";
        ret += 1;
      }
      if (complex_pole_imag_.size() != complex_pair_count)
      {
        Log::error() << "RationalApprox: complex pole real and imaginary vector sizes must match\n";
        ret += 1;
      }
      if (complex_input_couplings_real_.size() != complex_mode_vector_size)
      {
        Log::error() << "RationalApprox: complex_b_real size must equal complex pair count * dimension\n";
        ret += 1;
      }
      if (complex_input_couplings_imag_.size() != complex_mode_vector_size)
      {
        Log::error() << "RationalApprox: complex_b_imag size must equal complex pair count * dimension\n";
        ret += 1;
      }
      if (complex_output_residues_real_.size() != complex_mode_vector_size)
      {
        Log::error() << "RationalApprox: complex_c_real size must equal complex pair count * dimension\n";
        ret += 1;
      }
      if (complex_output_residues_imag_.size() != complex_mode_vector_size)
      {
        Log::error() << "RationalApprox: complex_c_imag size must equal complex pair count * dimension\n";
        ret += 1;
      }

      for (auto pole : poles_)
      {
        if (pole == 0.0)
        {
          Log::error() << "RationalApprox: real poles must be nonzero\n";
          ret += 1;
        }
      }
      for (size_t pair = 0; pair < complex_pole_imag_.size(); ++pair)
      {
        if (complex_pole_imag_[pair] <= 0.0)
        {
          Log::error() << "RationalApprox: complex pole imaginary parts must be positive\n";
          ret += 1;
        }
        if (pair < complex_pole_real_.size()
            && complex_pole_real_[pair] == 0.0
            && complex_pole_imag_[pair] == 0.0)
        {
          Log::error() << "RationalApprox: complex poles must be nonzero\n";
          ret += 1;
        }
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::dimension() const -> size_t
    {
      return dimension_;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::realPoleCount() const -> size_t
    {
      return poles_.size();
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::complexPairCount() const -> size_t
    {
      return complex_pole_real_.size();
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::modeCount() const -> size_t
    {
      return realPoleCount() + complexPairCount();
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::stateCount() const -> size_t
    {
      return realPoleCount() + static_cast<size_t>(2) * complexPairCount();
    }

    template <class ScalarT, typename IdxT>
    bool RationalApprox<ScalarT, IdxT>::hasDerivativeFeedthrough() const
    {
      for (auto value : e_)
      {
        if (value != 0.0)
        {
          return true;
        }
      }
      return false;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::stateJacobianEntryCount() const -> size_t
    {
      return realPoleCount() * (dimension_ + static_cast<size_t>(1))
             + complexPairCount() * (static_cast<size_t>(2) * dimension_ + static_cast<size_t>(4));
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::outputJacobianEntryCount() const -> size_t
    {
      return dimension_ * dimension_ + dimension_ * stateCount();
    }

    template <class ScalarT, typename IdxT>
    void RationalApprox<ScalarT, IdxT>::initialize(const ScalarT* u0,
                                                   const ScalarT* up0,
                                                   ScalarT*       x,
                                                   ScalarT*       xp) const
    {
      for (size_t k = 0; k < realPoleCount(); ++k)
      {
        ScalarT input_value{0};
        ScalarT input_derivative{0};
        for (size_t i = 0; i < dimension_; ++i)
        {
          const auto coupling_index  = modeVectorIndex(k, i);
          input_value               += input_couplings_[coupling_index] * u0[i];
          input_derivative          += input_couplings_[coupling_index] * up0[i];
        }

        const auto pole       = poles_[k];
        x[realStateIndex(k)]  = -input_value / pole - input_derivative / (pole * pole);
        xp[realStateIndex(k)] = input_value + pole * x[realStateIndex(k)];
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
          input_value_real          += complex_input_couplings_real_[coupling_index] * u0[i];
          input_value_imag          += complex_input_couplings_imag_[coupling_index] * u0[i];
          input_derivative_real     += complex_input_couplings_real_[coupling_index] * up0[i];
          input_derivative_imag     += complex_input_couplings_imag_[coupling_index] * up0[i];
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

        x[real_state_index] = -(input_value_real * inv_pole_real - input_value_imag * inv_pole_imag)
                              - (input_derivative_real * inv_pole_sq_real
                                 - input_derivative_imag * inv_pole_sq_imag);
        x[imag_state_index] = -(input_value_real * inv_pole_imag + input_value_imag * inv_pole_real)
                              - (input_derivative_real * inv_pole_sq_imag
                                 + input_derivative_imag * inv_pole_sq_real);

        xp[real_state_index] = input_value_real
                               + pole_real * x[real_state_index]
                               - pole_imag * x[imag_state_index];
        xp[imag_state_index] = input_value_imag
                               + pole_imag * x[real_state_index]
                               + pole_real * x[imag_state_index];
      }
    }

    template <class ScalarT, typename IdxT>
    void RationalApprox<ScalarT, IdxT>::evaluateStateResidual(const ScalarT* u,
                                                              const ScalarT* x,
                                                              const ScalarT* xp,
                                                              ScalarT*       f) const
    {
      for (size_t k = 0; k < realPoleCount(); ++k)
      {
        ScalarT input_value{0};
        for (size_t i = 0; i < dimension_; ++i)
        {
          input_value += input_couplings_[modeVectorIndex(k, i)] * u[i];
        }

        f[realStateIndex(k)] = -xp[realStateIndex(k)] + input_value + poles_[k] * x[realStateIndex(k)];
      }

      for (size_t pair = 0; pair < complexPairCount(); ++pair)
      {
        ScalarT input_value_real{0};
        ScalarT input_value_imag{0};
        for (size_t i = 0; i < dimension_; ++i)
        {
          const auto coupling_index  = modeVectorIndex(pair, i);
          input_value_real          += complex_input_couplings_real_[coupling_index] * u[i];
          input_value_imag          += complex_input_couplings_imag_[coupling_index] * u[i];
        }

        const auto real_state_index = complexRealStateIndex(pair);
        const auto imag_state_index = complexImagStateIndex(pair);
        const auto pole_real        = complex_pole_real_[pair];
        const auto pole_imag        = complex_pole_imag_[pair];

        f[real_state_index] = -xp[real_state_index]
                              + input_value_real
                              + pole_real * x[real_state_index]
                              - pole_imag * x[imag_state_index];
        f[imag_state_index] = -xp[imag_state_index]
                              + input_value_imag
                              + pole_imag * x[real_state_index]
                              + pole_real * x[imag_state_index];
      }
    }

    template <class ScalarT, typename IdxT>
    void RationalApprox<ScalarT, IdxT>::evaluateOutput(const ScalarT* u,
                                                       const ScalarT* up,
                                                       const ScalarT* x,
                                                       ScalarT*       z) const
    {
      for (size_t i = 0; i < dimension_; ++i)
      {
        ScalarT output{0};
        for (size_t j = 0; j < dimension_; ++j)
        {
          const auto coefficient_index  = matrixIndex(i, j);
          output                       += d_[coefficient_index] * u[j];
          output                       += e_[coefficient_index] * up[j];
        }

        for (size_t k = 0; k < realPoleCount(); ++k)
        {
          output += output_residues_[modeVectorIndex(k, i)] * x[realStateIndex(k)];
        }
        for (size_t pair = 0; pair < complexPairCount(); ++pair)
        {
          const auto residue_index  = modeVectorIndex(pair, i);
          output                   += static_cast<RealT>(2.0)
                    * (complex_output_residues_real_[residue_index] * x[complexRealStateIndex(pair)]
                       - complex_output_residues_imag_[residue_index] * x[complexImagStateIndex(pair)]);
        }

        z[i] = output;
      }
    }

    template <class ScalarT, typename IdxT>
    template <class AddEntry>
    void RationalApprox<ScalarT, IdxT>::addStateJacobianEntries(IdxT       row0,
                                                                IdxT       input_col0,
                                                                IdxT       state_col0,
                                                                RealT      alpha,
                                                                AddEntry&& add_entry) const
    {
      for (size_t k = 0; k < realPoleCount(); ++k)
      {
        const auto state_index = realStateIndex(k);
        for (size_t i = 0; i < dimension_; ++i)
        {
          add_entry(row0 + static_cast<IdxT>(state_index),
                    input_col0 + static_cast<IdxT>(i),
                    input_couplings_[modeVectorIndex(k, i)]);
        }
        add_entry(row0 + static_cast<IdxT>(state_index),
                  state_col0 + static_cast<IdxT>(state_index),
                  poles_[k] - alpha);
      }

      for (size_t pair = 0; pair < complexPairCount(); ++pair)
      {
        const auto real_state_index = complexRealStateIndex(pair);
        const auto imag_state_index = complexImagStateIndex(pair);

        for (size_t i = 0; i < dimension_; ++i)
        {
          add_entry(row0 + static_cast<IdxT>(real_state_index),
                    input_col0 + static_cast<IdxT>(i),
                    complex_input_couplings_real_[modeVectorIndex(pair, i)]);
        }
        add_entry(row0 + static_cast<IdxT>(real_state_index),
                  state_col0 + static_cast<IdxT>(real_state_index),
                  complex_pole_real_[pair] - alpha);
        add_entry(row0 + static_cast<IdxT>(real_state_index),
                  state_col0 + static_cast<IdxT>(imag_state_index),
                  -complex_pole_imag_[pair]);

        for (size_t i = 0; i < dimension_; ++i)
        {
          add_entry(row0 + static_cast<IdxT>(imag_state_index),
                    input_col0 + static_cast<IdxT>(i),
                    complex_input_couplings_imag_[modeVectorIndex(pair, i)]);
        }
        add_entry(row0 + static_cast<IdxT>(imag_state_index),
                  state_col0 + static_cast<IdxT>(real_state_index),
                  complex_pole_imag_[pair]);
        add_entry(row0 + static_cast<IdxT>(imag_state_index),
                  state_col0 + static_cast<IdxT>(imag_state_index),
                  complex_pole_real_[pair] - alpha);
      }
    }

    template <class ScalarT, typename IdxT>
    template <class AddEntry>
    void RationalApprox<ScalarT, IdxT>::addOutputJacobianEntries(IdxT       row0,
                                                                 IdxT       input_col0,
                                                                 IdxT       state_col0,
                                                                 RealT      alpha,
                                                                 RealT      scale,
                                                                 AddEntry&& add_entry) const
    {
      for (size_t i = 0; i < dimension_; ++i)
      {
        for (size_t j = 0; j < dimension_; ++j)
        {
          const auto coefficient_index = matrixIndex(i, j);
          add_entry(row0 + static_cast<IdxT>(i),
                    input_col0 + static_cast<IdxT>(j),
                    scale * (d_[coefficient_index] + alpha * e_[coefficient_index]));
        }

        for (size_t k = 0; k < realPoleCount(); ++k)
        {
          add_entry(row0 + static_cast<IdxT>(i),
                    state_col0 + static_cast<IdxT>(realStateIndex(k)),
                    scale * output_residues_[modeVectorIndex(k, i)]);
        }
        for (size_t pair = 0; pair < complexPairCount(); ++pair)
        {
          const auto residue_index = modeVectorIndex(pair, i);
          add_entry(row0 + static_cast<IdxT>(i),
                    state_col0 + static_cast<IdxT>(complexRealStateIndex(pair)),
                    scale * static_cast<RealT>(2.0) * complex_output_residues_real_[residue_index]);
          add_entry(row0 + static_cast<IdxT>(i),
                    state_col0 + static_cast<IdxT>(complexImagStateIndex(pair)),
                    -scale * static_cast<RealT>(2.0) * complex_output_residues_imag_[residue_index]);
        }
      }
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::matrixIndex(size_t row, size_t col) const -> size_t
    {
      return row * dimension_ + col;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::modeVectorIndex(size_t mode, size_t component) const -> size_t
    {
      return mode * dimension_ + component;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::realStateIndex(size_t mode) const -> size_t
    {
      return mode;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::complexRealStateIndex(size_t pair) const -> size_t
    {
      return realPoleCount() + static_cast<size_t>(2) * pair;
    }

    template <class ScalarT, typename IdxT>
    auto RationalApprox<ScalarT, IdxT>::complexImagStateIndex(size_t pair) const -> size_t
    {
      return complexRealStateIndex(pair) + 1;
    }

  } // namespace EMT
} // namespace GridKit
