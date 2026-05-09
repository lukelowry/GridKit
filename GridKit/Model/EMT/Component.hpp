/**
 * @file Component.hpp
 * @brief Base class for EMT network components.
 */

#pragma once

#include <stdexcept>
#include <vector>

#include <GridKit/LinearAlgebra/SparseMatrix/COO_Matrix.hpp>
#include <GridKit/LinearAlgebra/SparseMatrix/CsrMatrix.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename IdxT>
    struct IndexRange
    {
      IdxT begin{0};
      IdxT size{0};
    };

    enum class Action
    {
      Off,
      On
    };

    template <class ScalarT, typename IdxT>
    class Component
    {
    public:
      using RealT      = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using MatrixT    = GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT>;
      using CsrMatrixT = GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>;

      struct TerminalView
      {
        const ScalarT* y{nullptr};
        const ScalarT* yp{nullptr};
        IdxT           variable_index{0};
        IdxT           residual_index{0};
      };

      struct TerminalRef
      {
        Component<ScalarT, IdxT>* component{nullptr};
        size_t                    index{0};
      };

      Component() = default;

      virtual ~Component()
      {
        if (J_rows_buffer_ != nullptr)
        {
          delete[] J_rows_buffer_;
          delete[] J_cols_buffer_;
          delete[] J_vals_buffer_;
        }
      }

      virtual int allocate()          = 0;
      virtual int initialize()        = 0;
      virtual int tagDifferentiable() = 0;
      virtual int evaluateResidual()  = 0;
      virtual int evaluateJacobian()  = 0;

      virtual int verify() const
      {
        return 0;
      }

      virtual auto terminalCount() const -> size_t
      {
        return 0;
      }

      auto terminal(size_t index) -> TerminalRef
      {
        return {this, index};
      }

      auto port(size_t index) -> TerminalRef
      {
        return terminal(index);
      }

      virtual int bindTerminal(size_t, TerminalView)
      {
        return 1;
      }

      virtual const ScalarT* terminalCurrent(size_t) const
      {
        return nullptr;
      }

      virtual bool terminalHasDerivativeFeedthrough(size_t) const
      {
        return false;
      }

      virtual void apply(Action)
      {
        throw std::runtime_error("EMT component does not accept cues");
      }

      virtual void stampResidual(std::vector<ScalarT>& residual) const
      {
        const auto range = residualRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          residual[static_cast<size_t>(range.begin + i)] += f_[static_cast<size_t>(i)];
        }
      }

      virtual void stampDifferentiable(std::vector<bool>& tag) const
      {
        const auto range = variableRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          const auto global_index = static_cast<size_t>(range.begin + i);
          tag[global_index]       = tag[global_index] || tag_[static_cast<size_t>(i)];
        }
      }

      bool hasJacobian()
      {
        return true;
      }

      void updateTime(RealT t, RealT a)
      {
        time_  = t;
        alpha_ = a;
      }

      void setIndexRanges(IndexRange<IdxT> variable_range, IndexRange<IdxT> residual_range)
      {
        variable_range_ = variable_range;
        residual_range_ = residual_range;
      }

      auto variableRange() const -> IndexRange<IdxT>
      {
        return variable_range_;
      }

      auto residualRange() const -> IndexRange<IdxT>
      {
        return residual_range_;
      }

      auto variableIndex(size_t local_index) const -> IdxT
      {
        return variable_range_.begin + static_cast<IdxT>(local_index);
      }

      auto residualIndex(size_t local_index) const -> IdxT
      {
        return residual_range_.begin + static_cast<IdxT>(local_index);
      }

      auto size() const -> IdxT
      {
        return size_;
      }

      auto nnz() const -> IdxT
      {
        return nnz_;
      }

      std::vector<ScalarT>& y()
      {
        return y_;
      }

      const std::vector<ScalarT>& y() const
      {
        return y_;
      }

      std::vector<ScalarT>& yp()
      {
        return yp_;
      }

      const std::vector<ScalarT>& yp() const
      {
        return yp_;
      }

      std::vector<bool>& tag()
      {
        return tag_;
      }

      const std::vector<bool>& tag() const
      {
        return tag_;
      }

      std::vector<ScalarT>& getResidual()
      {
        return f_;
      }

      const std::vector<ScalarT>& getResidual() const
      {
        return f_;
      }

      MatrixT& getJacobian()
      {
        return J_;
      }

      const MatrixT& getJacobian() const
      {
        return J_;
      }

    protected:
      void allocateVectors()
      {
        const auto size = static_cast<size_t>(size_);
        y_.resize(size);
        yp_.resize(size);
        f_.resize(size);
        tag_.resize(size);
      }

    protected:
      IdxT size_{0};
      IdxT nnz_{0};

      IndexRange<IdxT> variable_range_;
      IndexRange<IdxT> residual_range_;

      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<bool>    tag_;
      std::vector<ScalarT> f_;

      MatrixT J_;
      IdxT*   J_rows_buffer_{nullptr};
      IdxT*   J_cols_buffer_{nullptr};
      RealT*  J_vals_buffer_{nullptr};

      RealT time_{0.0};
      RealT alpha_{0.0};
    };

  } // namespace EMT
} // namespace GridKit
