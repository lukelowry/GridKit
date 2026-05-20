#pragma once

#include <algorithm>
#include <cstddef>

#include <GridKit/Model/EMT/Math/RationalApprox/RationalApprox.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Math
    {
      template <class ScalarT, typename IdxT>
      RationalApprox<ScalarT, IdxT>::RationalApprox(const DataT& data)
        : data_(data)
      {
      }

      template <class ScalarT, typename IdxT>
      void RationalApprox<ScalarT, IdxT>::setData(const DataT& data)
      {
        data_ = data;
      }

      template <class ScalarT, typename IdxT>
      const typename RationalApprox<ScalarT, IdxT>::DataT& RationalApprox<ScalarT, IdxT>::data() const
      {
        return data_;
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::dimension() const
      {
        return data_.dimension;
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::matrixSize() const
      {
        return data_.dimension * data_.dimension;
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::realPoleCount() const
      {
        return static_cast<IdxT>(data_.real_poles.size());
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::complexPairCount() const
      {
        return static_cast<IdxT>(data_.pair_real.size());
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::stateCount() const
      {
        return data_.dimension * (realPoleCount() + IdxT{2} * complexPairCount());
      }

      template <class ScalarT, typename IdxT>
      IdxT RationalApprox<ScalarT, IdxT>::equationCount() const
      {
        return stateCount();
      }

      template <class ScalarT, typename IdxT>
      bool RationalApprox<ScalarT, IdxT>::hasDerivativeFeedthrough() const
      {
        for (const auto& value : data_.e)
        {
          if (value != RealT{0.0})
          {
            return true;
          }
        }
        return false;
      }

      template <class ScalarT, typename IdxT>
      int RationalApprox<ScalarT, IdxT>::verify() const
      {
        if (data_.dimension == IdxT{0})
        {
          return 1;
        }

        const std::size_t n2 = static_cast<std::size_t>(matrixSize());
        if (data_.d.size() != n2 || data_.e.size() != n2)
        {
          return 1;
        }

        if (data_.real_residues.size() != data_.real_poles.size() * n2)
        {
          return 1;
        }

        if (data_.pair_real.size() != data_.pair_imag.size())
        {
          return 1;
        }

        if (data_.pair_residue_real.size() != data_.pair_real.size() * n2)
        {
          return 1;
        }

        if (data_.pair_residue_imag.size() != data_.pair_real.size() * n2)
        {
          return 1;
        }

        for (const auto& pole : data_.real_poles)
        {
          if (pole == RealT{0.0})
          {
            return 1;
          }
        }

        for (const auto& omega : data_.pair_imag)
        {
          if (omega <= RealT{0.0})
          {
            return 1;
          }
        }

        return 0;
      }

      template <class ScalarT, typename IdxT>
      int RationalApprox<ScalarT, IdxT>::initialize(const ScalarT*, const ScalarT*, ScalarT* x, ScalarT* xp) const
      {
        if (verify() != 0)
        {
          return 1;
        }

        const std::size_t states = static_cast<std::size_t>(stateCount());
        if (x != nullptr)
        {
          std::fill(x, x + states, ScalarT{0.0});
        }
        if (xp != nullptr)
        {
          std::fill(xp, xp + states, ScalarT{0.0});
        }
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int RationalApprox<ScalarT, IdxT>::evaluateStateResidual(const ScalarT*,
                                                               const ScalarT*,
                                                               const ScalarT*,
                                                               ScalarT* f) const
      {
        if (verify() != 0)
        {
          return 1;
        }

        if (f != nullptr)
        {
          const std::size_t states = static_cast<std::size_t>(stateCount());
          std::fill(f, f + states, ScalarT{0.0});
        }
        return 0;
      }

      template <class ScalarT, typename IdxT>
      int RationalApprox<ScalarT, IdxT>::evaluateOutput(const ScalarT*,
                                                        const ScalarT*,
                                                        const ScalarT*,
                                                        ScalarT* z) const
      {
        if (verify() != 0)
        {
          return 1;
        }

        if (z != nullptr)
        {
          std::fill(z, z + static_cast<std::size_t>(data_.dimension), ScalarT{0.0});
        }
        return 0;
      }
    } // namespace Math
  } // namespace EMT
} // namespace GridKit
