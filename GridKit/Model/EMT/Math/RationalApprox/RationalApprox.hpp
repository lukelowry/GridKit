#pragma once

#include <cstddef>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Math/RationalApprox/RationalApproxData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Math
    {
      template <typename RealT>
      class RationalApprox
      {
      public:
        explicit RationalApprox(RationalApproxData<RealT> data)
          : data_(std::move(data))
        {
          validate();
        }

        std::size_t dimension() const
        {
          return data_.dimension;
        }

        std::size_t stateCount() const
        {
          return data_.dimension * (data_.real_poles.size() + 2 * data_.pair_real.size());
        }

        std::size_t equationCount() const
        {
          return stateCount();
        }

        bool hasDerivativeFeedthrough() const
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

        template <class ScalarT, class InputT, class InputDerivativeT>
        void initialize(std::span<ScalarT>      y,
                        std::span<ScalarT>      yp,
                        const InputT&           u0,
                        const InputDerivativeT& up0) const
        {
          requireStateSpan(y, "RationalApprox initial state");
          requireStateSpan(yp, "RationalApprox initial derivative");
          requireVectorSize(u0, "RationalApprox initial input");
          requireVectorSize(up0, "RationalApprox initial input derivative");

          const std::size_t n = dimension();
          std::size_t       offset{0};

          for (std::size_t m = 0; m < data_.real_poles.size(); ++m)
          {
            const RealT p  = data_.real_poles[m];
            const RealT p2 = p * p;
            for (std::size_t k = 0; k < n; ++k)
            {
              const auto x0  = -u0[k] / p - up0[k] / p2;
              y[offset + k]  = x0;
              yp[offset + k] = u0[k] + p * x0;
            }
            offset += n;
          }

          for (std::size_t q = 0; q < data_.pair_real.size(); ++q)
          {
            const RealT a     = data_.pair_real[q];
            const RealT omega = data_.pair_imag[q];
            const RealT den   = a * a + omega * omega;
            const RealT den2  = den * den;

            for (std::size_t k = 0; k < n; ++k)
            {
              const auto xr = -u0[k] * a / den - up0[k] * (a * a - omega * omega) / den2;
              const auto xi = u0[k] * omega / den + up0[k] * (RealT{2.0} * a * omega) / den2;

              y[offset + k]      = xr;
              y[offset + n + k]  = xi;
              yp[offset + k]     = u0[k] + a * xr - omega * xi;
              yp[offset + n + k] = omega * xr + a * xi;
            }
            offset += 2 * n;
          }
        }

        template <class ScalarT, class InputT, class InputDerivativeT, class OutputT>
        void residual(std::span<const ScalarT> y,
                      std::span<const ScalarT> yp,
                      std::span<ScalarT>       f,
                      const InputT&            u,
                      const InputDerivativeT&  up,
                      OutputT&                 z) const
        {
          requireStateSpan(y, "RationalApprox state");
          requireStateSpan(yp, "RationalApprox derivative");
          requireStateSpan(f, "RationalApprox residual");
          requireVectorSize(u, "RationalApprox input");
          requireVectorSize(up, "RationalApprox input derivative");
          requireVectorSize(z, "RationalApprox output");

          const std::size_t n = dimension();
          zero(z);
          addMatrixVector(data_.d, 0, u, z);
          addMatrixVector(data_.e, 0, up, z);

          std::size_t state_offset{0};

          for (std::size_t m = 0; m < data_.real_poles.size(); ++m)
          {
            const RealT p = data_.real_poles[m];
            for (std::size_t k = 0; k < n; ++k)
            {
              f[state_offset + k] = -yp[state_offset + k] + u[k] + p * y[state_offset + k];
            }
            addMatrixVector(data_.real_residues, m * matrixSize(), y.subspan(state_offset, n), z);
            state_offset += n;
          }

          for (std::size_t q = 0; q < data_.pair_real.size(); ++q)
          {
            const RealT       a         = data_.pair_real[q];
            const RealT       omega     = data_.pair_imag[q];
            const std::size_t real_base = state_offset;
            const std::size_t imag_base = state_offset + n;

            for (std::size_t k = 0; k < n; ++k)
            {
              const auto xr    = y[real_base + k];
              const auto xi    = y[imag_base + k];
              f[real_base + k] = -yp[real_base + k] + u[k] + a * xr - omega * xi;
              f[imag_base + k] = -yp[imag_base + k] + omega * xr + a * xi;
            }

            addMatrixVector(data_.pair_residue_real,
                            q * matrixSize(),
                            y.subspan(real_base, n),
                            z,
                            RealT{2.0});
            addMatrixVector(data_.pair_residue_imag,
                            q * matrixSize(),
                            y.subspan(imag_base, n),
                            z,
                            RealT{-2.0});
            state_offset += 2 * n;
          }
        }

      private:
        std::size_t matrixSize() const
        {
          return data_.dimension * data_.dimension;
        }

        void validate() const
        {
          if (data_.dimension == 0)
          {
            throw std::invalid_argument("RationalApprox dimension must be positive");
          }

          const std::size_t n2 = matrixSize();
          requireSize(data_.d, n2, "RationalApprox D matrix");
          requireSize(data_.e, n2, "RationalApprox E matrix");
          requireSize(data_.real_residues,
                      data_.real_poles.size() * n2,
                      "RationalApprox real-pole residues");
          requireSize(data_.pair_real, data_.pair_imag.size(), "RationalApprox complex-pair real poles");
          requireSize(data_.pair_residue_real,
                      data_.pair_real.size() * n2,
                      "RationalApprox complex-pair real residues");
          requireSize(data_.pair_residue_imag,
                      data_.pair_real.size() * n2,
                      "RationalApprox complex-pair imaginary residues");

          for (const auto& pole : data_.real_poles)
          {
            if (pole == RealT{0.0})
            {
              throw std::invalid_argument("RationalApprox real poles must be nonzero");
            }
          }
          for (const auto& omega : data_.pair_imag)
          {
            if (omega <= RealT{0.0})
            {
              throw std::invalid_argument("RationalApprox complex-pair imaginary poles must be positive");
            }
          }
        }

        template <class VectorT>
        void requireStateSpan(const VectorT& vector, const char* name) const
        {
          requireSize(vector, stateCount(), name);
        }

        template <class VectorT>
        void requireVectorSize(const VectorT& vector, const char* name) const
        {
          requireSize(vector, dimension(), name);
        }

        template <class VectorT>
        static void requireSize(const VectorT& vector, std::size_t expected, const char* name)
        {
          if (vector.size() != expected)
          {
            throw std::invalid_argument(name);
          }
        }

        template <class OutputT>
        void zero(OutputT& z) const
        {
          for (std::size_t row = 0; row < dimension(); ++row)
          {
            z[row] = typename OutputT::value_type{0.0};
          }
        }

        template <class InputT, class OutputT>
        void addMatrixVector(const std::vector<RealT>& matrix,
                             std::size_t               matrix_offset,
                             const InputT&             input,
                             OutputT&                  output,
                             RealT                     scale = RealT{1.0}) const
        {
          const std::size_t n = dimension();
          for (std::size_t row = 0; row < n; ++row)
          {
            for (std::size_t col = 0; col < n; ++col)
            {
              output[row] += (scale * matrix[matrix_offset + row * n + col]) * input[col];
            }
          }
        }

        RationalApproxData<RealT> data_;
      };
    } // namespace Math
  } // namespace EMT
} // namespace GridKit
