#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <GridKit/LinearAlgebra/SparseMatrix/CsrMatrix.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class Assembly
    {
    public:
      using Entry      = std::pair<IdxT, IdxT>;
      using CsrMatrixT = GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>;

      void build(IdxT size, std::vector<Entry> entries)
      {
        size_ = size;

        std::sort(entries.begin(), entries.end());
        entries.erase(std::unique(entries.begin(), entries.end()), entries.end());

        row_ptrs_.assign(static_cast<size_t>(size_) + 1, IdxT{0});
        for (const auto& [row, col] : entries)
        {
          if (row >= size_ || col >= size_)
          {
            throw std::out_of_range("EMT assembly pattern entry is out of range");
          }
          ++row_ptrs_[row + 1];
        }
        for (IdxT row = 0; row < size_; ++row)
        {
          row_ptrs_[row + 1] += row_ptrs_[row];
        }

        cols_.assign(entries.size(), IdxT{0});
        values_.assign(entries.size(), RealT{0.0});

        std::vector<IdxT> next = row_ptrs_;
        for (const auto& [row, col] : entries)
        {
          cols_[next[row]++] = col;
        }

        csr_ = std::make_unique<CsrMatrixT>(size_, size_, static_cast<IdxT>(entries.size()));
        csr_->setDataPointers(row_ptrs_.data(), cols_.data(), values_.data(), GridKit::LinearAlgebra::memory::HOST);
      }

      void clearValues()
      {
        std::fill(values_.begin(), values_.end(), RealT{0.0});
        if (csr_)
        {
          csr_->setUpdated(GridKit::LinearAlgebra::memory::HOST);
        }
      }

      IdxT size() const
      {
        return size_;
      }

      IdxT nnz() const
      {
        return static_cast<IdxT>(values_.size());
      }

      RealT* values()
      {
        return values_.data();
      }

      const RealT* values() const
      {
        return values_.data();
      }

      const IdxT* rowPtrs() const
      {
        return row_ptrs_.data();
      }

      const IdxT* columns() const
      {
        return cols_.data();
      }

      CsrMatrixT* matrix() const
      {
        return csr_.get();
      }

    private:
      IdxT                        size_{0};
      std::vector<IdxT>           row_ptrs_;
      std::vector<IdxT>           cols_;
      std::vector<RealT>          values_;
      std::unique_ptr<CsrMatrixT> csr_;
    };
  } // namespace EMT
} // namespace GridKit
