#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace GridKit
{
  namespace EMT
  {
    template <class T>
    using PhaseVector = std::array<T, 3>;

    template <class T>
    using PhaseMatrix = std::array<PhaseVector<T>, 3>;

    template <class T>
    bool isFinite(const PhaseMatrix<T>& matrix)
    {
      for (const auto& row : matrix)
      {
        for (const auto& value : row)
        {
          if (!std::isfinite(value))
          {
            return false;
          }
        }
      }
      return true;
    }

    template <class T>
    PhaseMatrix<T> scale(const PhaseMatrix<T>& matrix, T factor)
    {
      PhaseMatrix<T> result{};
      for (std::size_t row = 0; row < 3; ++row)
      {
        for (std::size_t col = 0; col < 3; ++col)
        {
          result[row][col] = factor * matrix[row][col];
        }
      }
      return result;
    }

    template <class MatrixScalar, class VectorScalar>
    auto multiply(const PhaseMatrix<MatrixScalar>& matrix, const PhaseVector<VectorScalar>& vector)
    {
      using Scalar = std::remove_cv_t<decltype(std::declval<MatrixScalar>() * std::declval<VectorScalar>())>;

      PhaseVector<Scalar> result{};
      for (std::size_t row = 0; row < 3; ++row)
      {
        for (std::size_t col = 0; col < 3; ++col)
        {
          result[row] += static_cast<Scalar>(matrix[row][col]) * vector[col];
        }
      }
      return result;
    }

    template <class T>
    T determinant(const PhaseMatrix<T>& matrix)
    {
      return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
             - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
             + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }

    template <class T>
    bool isSingular(const PhaseMatrix<T>& matrix)
    {
      return std::abs(determinant(matrix)) <= std::numeric_limits<T>::epsilon();
    }

    template <class T>
    PhaseVector<T> solve(PhaseMatrix<T> matrix, PhaseVector<T> rhs)
    {
      using Real = decltype(std::abs(std::declval<T>()));

      for (std::size_t pivot = 0; pivot < 3; ++pivot)
      {
        std::size_t row_with_pivot = pivot;
        Real        pivot_norm     = std::abs(matrix[pivot][pivot]);
        for (std::size_t row = pivot + 1; row < 3; ++row)
        {
          const Real candidate_norm = std::abs(matrix[row][pivot]);
          if (candidate_norm > pivot_norm)
          {
            row_with_pivot = row;
            pivot_norm     = candidate_norm;
          }
        }

        if (pivot_norm <= std::numeric_limits<Real>::epsilon())
        {
          throw std::invalid_argument("EMT phase matrix is singular");
        }

        if (row_with_pivot != pivot)
        {
          std::swap(matrix[pivot], matrix[row_with_pivot]);
          std::swap(rhs[pivot], rhs[row_with_pivot]);
        }

        for (std::size_t row = pivot + 1; row < 3; ++row)
        {
          const T factor     = matrix[row][pivot] / matrix[pivot][pivot];
          matrix[row][pivot] = T{0.0};
          for (std::size_t col = pivot + 1; col < 3; ++col)
          {
            matrix[row][col] -= factor * matrix[pivot][col];
          }
          rhs[row] -= factor * rhs[pivot];
        }
      }

      PhaseVector<T> result{};
      for (std::size_t reverse = 0; reverse < 3; ++reverse)
      {
        const std::size_t row = 2 - reverse;
        T                 sum = rhs[row];
        for (std::size_t col = row + 1; col < 3; ++col)
        {
          sum -= matrix[row][col] * result[col];
        }
        result[row] = sum / matrix[row][row];
      }
      return result;
    }
  } // namespace EMT
} // namespace GridKit
