#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <GridKit/Definitions.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    int VectorFit<scalar_type, index_type, N, K>::evaluateJacobian()
    {
      readInput();
      J_.zeroMatrix();

      const auto y_out = yOutOffset();

      std::vector<IdxT>  rows;
      std::vector<IdxT>  cols;
      std::vector<RealT> vals;

      const std::size_t nnz =
          5 * pole_count_ * K + 2 * pole_count_ * N * K + N + N * K;
      rows.reserve(nnz);
      cols.reserve(nnz);
      vals.reserve(nnz);

      auto store = [&](std::size_t row, IdxT col, RealT val)
      {
        if (val == RealT{0.0})
        {
          return;
        }

        rows.push_back(residual_indices_[row]);
        cols.push_back(col);
        vals.push_back(val);
      };

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        const RealT a     = poles_[q][0];
        const RealT omega = poles_[q][1];

        const std::size_t w = 2 * q * K;
        const std::size_t v = w + K;

        for (std::size_t k = 0; k < K; ++k)
        {
          store(w + k, variable_indices_[w + k], a - alpha_);
          store(w + k, variable_indices_[v + k], -omega);
          store(w + k, ws_indices_[k], RealT{1.0});

          store(v + k, variable_indices_[w + k], omega);
          store(v + k, variable_indices_[v + k], a - alpha_);
        }
      }

      for (std::size_t n = 0; n < N; ++n)
      {
        const std::size_t row = y_out + n;

        store(row, variable_indices_[row], RealT{1.0});

        for (std::size_t k = 0; k < K; ++k)
        {
          store(row, ws_indices_[k], -D_(n, k) - alpha_ * E_(n, k));
        }
      }

      for (std::size_t q = 0; q < pole_count_; ++q)
      {
        const std::size_t w = 2 * q * K;
        const std::size_t v = w + K;

        for (std::size_t n = 0; n < N; ++n)
        {
          const std::size_t row = y_out + n;

          for (std::size_t k = 0; k < K; ++k)
          {
            store(row, variable_indices_[w + k], -A_[q](n, k));
            store(row, variable_indices_[v + k], B_[q](n, k));
          }
        }
      }

      J_.setValues(std::move(rows), std::move(cols), std::move(vals));

      return 0;
    }
  } // namespace EMT
} // namespace GridKit
#endif
