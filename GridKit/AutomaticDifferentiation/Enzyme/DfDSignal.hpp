/**
 * @file DfDSignal.hpp
 *
 * @brief EMT sparse Jacobian evaluator for a borrowed input block: df/du.
 * Columns are the producing element's global indices, taken from the input
 * Signal. Reuses the sparse storage plumbing in LowerSparseStorage.hpp.
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include <GridKit/AutomaticDifferentiation/Enzyme/EmtModelWrapper.hpp>
#include <GridKit/AutomaticDifferentiation/Enzyme/EnzymeDefinitions.hpp>
#include <GridKit/AutomaticDifferentiation/Enzyme/LowerSparseStorage.hpp>
#include <GridKit/LinearAlgebra/SparseMatrix/COO_Matrix.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace Enzyme
  {
    namespace Sparse
    {
      template <typename ModelT, class ScalarT, typename IdxT>
      struct DfDSignal
      {
        using RealT   = typename GridKit::ScalarTraits<ScalarT>::RealT;
        using MatrixT = GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT>;

        static void eval(ModelT*     model,
                         size_t      n_res,
                         size_t      n_var,
                         const IdxT* res_indices,
                         const IdxT* var_indices,
                         ScalarT*    y,
                         ScalarT*    yp,
                         ScalarT*    u,
                         RealT       scale,
                         IdxT*       rows,
                         IdxT*       cols,
                         RealT*      vals,
                         MatrixT&    jac)
        {
          if (n_res == 0 || n_var == 0)
          {
            return;
          }

          std::vector<ScalarT> residual(n_res);
          IdxT                 nnz = 0;

          for (size_t var_i = 0; var_i < n_var; ++var_i)
          {
            ScalarT* output   = __enzyme_todense<ScalarT*>((void*) ident_load<ScalarT, IdxT>,
                                                         (void*) ident_store<ScalarT, IdxT>,
                                                         var_i);
            ScalarT* d_output = __enzyme_todense<ScalarT*>((void*) sparse_load<ScalarT, IdxT>,
                                                           (void*) sparse_store<ScalarT, IdxT>,
                                                           var_i,
                                                           res_indices,
                                                           var_indices,
                                                           rows,
                                                           cols,
                                                           vals,
                                                           &nnz);

            std::ranges::fill(residual, 0.0);

            __enzyme_fwddiff<void>((void*) EmtModelWrapper<ModelT, ScalarT>::eval,
                                   enzyme_const,
                                   model,
                                   enzyme_const,
                                   y,
                                   enzyme_const,
                                   yp,
                                   enzyme_dup,
                                   u,
                                   output,
                                   enzyme_dupnoneed,
                                   residual.data(),
                                   d_output);
          }

          jac.setValues(scale, rows, cols, vals, nnz);
        }
      };
    } // namespace Sparse
  } // namespace Enzyme
} // namespace GridKit
