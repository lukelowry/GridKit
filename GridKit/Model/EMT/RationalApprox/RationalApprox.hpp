/**
 * @file RationalApprox.hpp
 * @brief Real state-space realization for vector-fitted matrix rational approximations.
 */

#pragma once

#include <cstddef>
#include <vector>

#include <GridKit/Model/EMT/RationalApprox/RationalApproxData.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    /**
     * @brief Non-component equation block for square rational matrix approximations.
     *
     * `RationalApprox` owns only the fitted coefficients and realization math.
     * It does not own GridKit variables, residuals, signals, or global indices.
     */
    template <class ScalarT, typename IdxT>
    class RationalApprox
    {
    public:
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using DataT = RationalApproxData<RealT, IdxT>;

      RationalApprox() = default;
      explicit RationalApprox(const DataT& data);

      void setData(const DataT& data);
      int  verify() const;

      auto dimension() const -> size_t;
      auto realPoleCount() const -> size_t;
      auto complexPairCount() const -> size_t;
      auto modeCount() const -> size_t;
      auto stateCount() const -> size_t;

      bool hasDerivativeFeedthrough() const;

      auto stateJacobianEntryCount() const -> size_t;
      auto outputJacobianEntryCount() const -> size_t;

      void initialize(const ScalarT* u0, const ScalarT* up0, ScalarT* x, ScalarT* xp) const;
      void evaluateStateResidual(const ScalarT* u, const ScalarT* x, const ScalarT* xp, ScalarT* f) const;
      void evaluateOutput(const ScalarT* u, const ScalarT* up, const ScalarT* x, ScalarT* z) const;

      template <class AddEntry>
      void addStateJacobianEntries(IdxT       row0,
                                   IdxT       input_col0,
                                   IdxT       state_col0,
                                   RealT      alpha,
                                   AddEntry&& add_entry) const;

      template <class AddEntry>
      void addOutputJacobianEntries(IdxT       row0,
                                    IdxT       input_col0,
                                    IdxT       state_col0,
                                    RealT      alpha,
                                    RealT      scale,
                                    AddEntry&& add_entry) const;

    private:
      auto matrixIndex(size_t row, size_t col) const -> size_t;
      auto modeVectorIndex(size_t mode, size_t component) const -> size_t;
      auto realStateIndex(size_t mode) const -> size_t;
      auto complexRealStateIndex(size_t pair) const -> size_t;
      auto complexImagStateIndex(size_t pair) const -> size_t;

    private:
      size_t dimension_{0};

      std::vector<RealT> d_;
      std::vector<RealT> e_;

      std::vector<RealT> poles_;
      std::vector<RealT> input_couplings_;
      std::vector<RealT> output_residues_;

      std::vector<RealT> complex_pole_real_;
      std::vector<RealT> complex_pole_imag_;
      std::vector<RealT> complex_input_couplings_real_;
      std::vector<RealT> complex_input_couplings_imag_;
      std::vector<RealT> complex_output_residues_real_;
      std::vector<RealT> complex_output_residues_imag_;
    };

  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/RationalApprox/RationalApproxImpl.hpp>
