#pragma once

#include <array>
#include <cstddef>
#include <type_traits>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Equation
    {
      template <class T, std::size_t N>
      struct Vec
      {
        using value_type                  = T;
        static constexpr std::size_t size = N;
        static_assert(N > 0, "Equation vectors must be non-empty");

        std::array<T, N> data{};

        __attribute__((always_inline)) T& operator[](std::size_t i)
        {
          return data[i];
        }

        __attribute__((always_inline)) const T& operator[](std::size_t i) const
        {
          return data[i];
        }
      };

      template <class T, std::size_t N>
      struct VecRef
      {
        using value_type                  = T;
        static constexpr std::size_t size = N;
        static_assert(N > 0, "Equation vectors must be non-empty");

        T* data{nullptr};

        __attribute__((always_inline)) T& operator[](std::size_t i)
        {
          return data[i];
        }

        __attribute__((always_inline)) const T& operator[](std::size_t i) const
        {
          return data[i];
        }

        __attribute__((always_inline)) VecRef& operator=(const VecRef& rhs)
        {
          for (std::size_t i = 0; i < N; ++i)
          {
            data[i] = rhs[i];
          }
          return *this;
        }

        template <class Expr>
        __attribute__((always_inline)) VecRef& operator=(const Expr& rhs)
        {
          static_assert(std::decay_t<Expr>::size == N, "Equation vector sizes must match");
          for (std::size_t i = 0; i < N; ++i)
          {
            data[i] = rhs[i];
          }
          return *this;
        }
      };

      template <class T, std::size_t R, std::size_t C>
      struct Mat
      {
        using value_type                  = T;
        static constexpr std::size_t rows = R;
        static constexpr std::size_t cols = C;
        static_assert(R > 0 && C > 0, "Equation matrices must be non-empty");

        std::array<T, R * C> data{};

        __attribute__((always_inline)) T& operator()(std::size_t r, std::size_t c)
        {
          return data[r * C + c];
        }

        __attribute__((always_inline)) const T& operator()(std::size_t r, std::size_t c) const
        {
          return data[r * C + c];
        }
      };

      template <std::size_t N, class T>
      __attribute__((always_inline)) inline VecRef<T, N> slice(T* base, std::size_t offset)
      {
        return VecRef<T, N>{base + offset};
      }

      template <std::size_t N, class T>
      __attribute__((always_inline)) inline VecRef<T, N> block(T* base, std::size_t block_index)
      {
        return VecRef<T, N>{base + block_index * N};
      }

      template <class L, class R>
      __attribute__((always_inline)) inline auto operator-(const L& lhs, const R& rhs)
      {
        using LeftT  = std::decay_t<L>;
        using RightT = std::decay_t<R>;
        static_assert(LeftT::size == RightT::size, "Equation vector sizes must match");

        using OutT = std::decay_t<decltype(lhs[0] - rhs[0])>;
        Vec<OutT, LeftT::size> out{};
        for (std::size_t i = 0; i < LeftT::size; ++i)
        {
          out[i] = lhs[i] - rhs[i];
        }
        return out;
      }

      template <class L, class R>
      __attribute__((always_inline)) inline auto operator+(const L& lhs, const R& rhs)
      {
        using LeftT  = std::decay_t<L>;
        using RightT = std::decay_t<R>;
        static_assert(LeftT::size == RightT::size, "Equation vector sizes must match");

        using OutT = std::decay_t<decltype(lhs[0] + rhs[0])>;
        Vec<OutT, LeftT::size> out{};
        for (std::size_t i = 0; i < LeftT::size; ++i)
        {
          out[i] = lhs[i] + rhs[i];
        }
        return out;
      }

      template <class T, std::size_t R, std::size_t C, class V>
      __attribute__((always_inline)) inline auto operator*(const Mat<T, R, C>& A, const V& x)
      {
        using VecT = std::decay_t<V>;
        static_assert(VecT::size == C, "Equation matrix/vector dimensions must match");

        using OutT = std::decay_t<decltype(A(0, 0) * x[0])>;
        Vec<OutT, R> out{};
        for (std::size_t r = 0; r < R; ++r)
        {
          for (std::size_t c = 0; c < C; ++c)
          {
            out[r] += A(r, c) * x[c];
          }
        }
        return out;
      }
    } // namespace Equation
  } // namespace PhasorDynamics
} // namespace GridKit
