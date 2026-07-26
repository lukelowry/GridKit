#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Detail
    {
      template <typename T>
      T determinant(const ABCMatrix<T>& A)
      {
        return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
               - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
               + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
      }

      template <typename T>
      ABCMatrix<T> inverse(const ABCMatrix<T>& A)
      {
        const T det = determinant(A);
        return {{{(A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det,
                  (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det,
                  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det},
                 {(A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det,
                  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det,
                  (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det},
                 {(A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det,
                  (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det,
                  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det}}};
      }

      template <typename RealT>
      bool finite(const ABCVector<RealT>& x)
      {
        return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
      }

      template <typename RealT>
      bool finite(const ABCMatrix<RealT>& A)
      {
        return finite(A[0]) && finite(A[1]) && finite(A[2]);
      }

      template <typename RealT, typename ScalarT>
      RealT scalarValue(const ScalarT& value)
      {
        if constexpr (requires { value.getValue(); })
        {
          return static_cast<RealT>(value.getValue());
        }
        else
        {
          return static_cast<RealT>(value);
        }
      }

      template <typename RealT>
      bool accumulateNormalizedRowGram(ABCMatrix<RealT>&       gram,
                                       const ABCMatrix<RealT>& block)
      {
        if (!finite(block))
        {
          return false;
        }

        for (std::size_t equation = 0; equation < 3; ++equation)
        {
          RealT row_scale{0.0};
          for (const auto value : block[equation])
          {
            row_scale = std::max(row_scale, std::abs(value));
          }
          if (row_scale == RealT{0.0})
          {
            continue;
          }

          for (std::size_t row = 0; row < 3; ++row)
          {
            const auto normalized_value = block[equation][row] / row_scale;
            for (std::size_t column = 0; column < 3; ++column)
            {
              gram[row][column] +=
                  normalized_value * (block[equation][column] / row_scale);
            }
          }
        }
        return true;
      }

      template <typename RealT>
      RealT matrixScale(const ABCMatrix<RealT>& A)
      {
        RealT scale{0.0};
        for (const auto& row : A)
        {
          for (const auto value : row)
          {
            scale = std::max(scale, std::abs(value));
          }
        }
        return scale;
      }

      template <typename RealT>
      bool zero(const ABCMatrix<RealT>& A)
      {
        for (const auto& row : A)
        {
          for (const auto value : row)
          {
            if (value != RealT{0.0})
            {
              return false;
            }
          }
        }
        return true;
      }

      template <typename RealT>
      bool symmetric(const ABCMatrix<RealT>& A)
      {
        const RealT tolerance = RealT{64.0} * std::numeric_limits<RealT>::epsilon() * matrixScale(A);
        return std::abs(A[0][1] - A[1][0]) <= tolerance
               && std::abs(A[0][2] - A[2][0]) <= tolerance
               && std::abs(A[1][2] - A[2][1]) <= tolerance;
      }

      template <typename RealT>
      bool positiveDefinite(const ABCMatrix<RealT>& A)
      {
        if (!finite(A) || !symmetric(A))
        {
          return false;
        }

        const RealT scale     = matrixScale(A);
        const RealT tolerance = RealT{64.0} * std::numeric_limits<RealT>::epsilon();
        const RealT minor2    = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        return scale > RealT{0.0}
               && A[0][0] > tolerance * scale
               && minor2 > tolerance * scale * scale
               && determinant(A) > tolerance * scale * scale * scale;
      }

      template <typename RealT>
      bool positiveSemidefinite(const ABCMatrix<RealT>& A)
      {
        if (!finite(A) || !symmetric(A))
        {
          return false;
        }

        const RealT scale     = matrixScale(A);
        const RealT tolerance = RealT{64.0} * std::numeric_limits<RealT>::epsilon();
        const RealT minor01   = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        const RealT minor02   = A[0][0] * A[2][2] - A[0][2] * A[2][0];
        const RealT minor12   = A[1][1] * A[2][2] - A[1][2] * A[2][1];

        return A[0][0] >= -tolerance * scale
               && A[1][1] >= -tolerance * scale
               && A[2][2] >= -tolerance * scale
               && minor01 >= -tolerance * scale * scale
               && minor02 >= -tolerance * scale * scale
               && minor12 >= -tolerance * scale * scale
               && determinant(A) >= -tolerance * scale * scale * scale;
      }

      template <typename RealT>
      std::size_t matrixRank(ABCMatrix<RealT> A)
      {
        const RealT scale = matrixScale(A);
        if (!finite(A) || scale == RealT{0.0})
        {
          return 0;
        }

        for (auto& row : A)
        {
          for (auto& value : row)
          {
            value /= scale;
          }
        }

        const RealT tolerance = RealT{128.0}
                                * std::numeric_limits<RealT>::epsilon();
        std::size_t rank{0};
        for (std::size_t column = 0; column < 3 && rank < 3; ++column)
        {
          std::size_t pivot = rank;
          for (std::size_t row = rank + 1; row < 3; ++row)
          {
            if (std::abs(A[row][column]) > std::abs(A[pivot][column]))
            {
              pivot = row;
            }
          }
          if (std::abs(A[pivot][column]) <= tolerance)
          {
            continue;
          }

          std::swap(A[rank], A[pivot]);
          for (std::size_t row = rank + 1; row < 3; ++row)
          {
            const RealT factor = A[row][column] / A[rank][column];
            for (std::size_t entry = column; entry < 3; ++entry)
            {
              A[row][entry] -= factor * A[rank][entry];
            }
          }
          ++rank;
        }
        return rank;
      }
    } // namespace Detail
  } // namespace EMT
} // namespace GridKit
