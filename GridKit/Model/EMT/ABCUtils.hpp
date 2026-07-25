#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
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
      T determinant(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22)
      {
        return a00 * (a11 * a22 - a12 * a21)
               - a01 * (a10 * a22 - a12 * a20)
               + a02 * (a10 * a21 - a11 * a20);
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
      void solveImpedance(const ABCMatrix<RealT>& R,
                          const ABCMatrix<RealT>& L,
                          RealT                   omega,
                          std::complex<RealT>     ba,
                          std::complex<RealT>     bb,
                          std::complex<RealT>     bc,
                          std::complex<RealT>&    xa,
                          std::complex<RealT>&    xb,
                          std::complex<RealT>&    xc)
      {
        const std::complex<RealT> s0{RealT{0.0}, omega};
        const auto                a00 = R[0][0] + s0 * L[0][0];
        const auto                a01 = R[0][1] + s0 * L[0][1];
        const auto                a02 = R[0][2] + s0 * L[0][2];
        const auto                a10 = R[1][0] + s0 * L[1][0];
        const auto                a11 = R[1][1] + s0 * L[1][1];
        const auto                a12 = R[1][2] + s0 * L[1][2];
        const auto                a20 = R[2][0] + s0 * L[2][0];
        const auto                a21 = R[2][1] + s0 * L[2][1];
        const auto                a22 = R[2][2] + s0 * L[2][2];
        const auto                det = determinant(a00, a01, a02, a10, a11, a12, a20, a21, a22);

        xa = determinant(ba, a01, a02, bb, a11, a12, bc, a21, a22)
             / det;
        xb = determinant(a00, ba, a02, a10, bb, a12, a20, bc, a22)
             / det;
        xc = determinant(a00, a01, ba, a10, a11, bb, a20, a21, bc)
             / det;
      }

      template <typename RealT>
      void multiplyAdmittance(const ABCMatrix<RealT>& G,
                              const ABCMatrix<RealT>& C,
                              RealT                   omega,
                              std::complex<RealT>     xa,
                              std::complex<RealT>     xb,
                              std::complex<RealT>     xc,
                              std::complex<RealT>&    ya,
                              std::complex<RealT>&    yb,
                              std::complex<RealT>&    yc)
      {
        const std::complex<RealT> s0{RealT{0.0}, omega};
        ya = (G[0][0] + s0 * C[0][0]) * xa
             + (G[0][1] + s0 * C[0][1]) * xb
             + (G[0][2] + s0 * C[0][2]) * xc;
        yb = (G[1][0] + s0 * C[1][0]) * xa
             + (G[1][1] + s0 * C[1][1]) * xb
             + (G[1][2] + s0 * C[1][2]) * xc;
        yc = (G[2][0] + s0 * C[2][0]) * xa
             + (G[2][1] + s0 * C[2][1]) * xb
             + (G[2][2] + s0 * C[2][2]) * xc;
      }

      template <typename ScalarT, typename RealT>
      void initializeSinusoid(std::complex<RealT> phasor_a,
                              std::complex<RealT> phasor_b,
                              std::complex<RealT> phasor_c,
                              RealT               omega,
                              ScalarT*            y,
                              ScalarT*            yp)
      {
        const RealT sqrt_two = std::sqrt(RealT{2.0});
        y[0]                 = sqrt_two * phasor_a.real();
        y[1]                 = sqrt_two * phasor_b.real();
        y[2]                 = sqrt_two * phasor_c.real();
        yp[0]                = -sqrt_two * omega * phasor_a.imag();
        yp[1]                = -sqrt_two * omega * phasor_b.imag();
        yp[2]                = -sqrt_two * omega * phasor_c.imag();
      }

      template <typename ScalarT, typename RealT>
      std::complex<RealT> phasor(ScalarT value, ScalarT derivative, RealT omega)
      {
        const RealT sqrt_two = std::sqrt(RealT{2.0});
        return {static_cast<RealT>(value) / sqrt_two,
                -static_cast<RealT>(derivative) / (sqrt_two * omega)};
      }
    } // namespace Detail
  } // namespace EMT
} // namespace GridKit
