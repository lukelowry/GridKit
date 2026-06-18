/**
 * @file DfDState.hpp
 *
 * @brief EMT sparse Jacobian evaluator for an element's owned state.
 * Algebraic callers differentiate only y. Differential callers add a separate
 * scaled y' pass so Enzyme never has to prove a no-store derivative block.
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
      struct DfDState
      {
        using RealT   = typename GridKit::ScalarTraits<ScalarT>::RealT;
        using MatrixT = GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT>;

        static void evalState(ModelT*     model,
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

          std::vector<ScalarT> elementary_v(n_var);
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

            std::ranges::fill(elementary_v, 0.0);
            elementary_v[var_i] = 1.0;

            __enzyme_fwddiff<void>((void*) EmtModelWrapper<ModelT, ScalarT>::eval,
                                   enzyme_const,
                                   model,
                                   enzyme_dup,
                                   y,
                                   output,
                                   enzyme_const,
                                   yp,
                                   enzyme_const,
                                   u,
                                   enzyme_dupnoneed,
                                   elementary_v.data(),
                                   d_output);
          }

          jac.setValues(scale, rows, cols, vals, nnz);
        }

        static void evalDerivative(ModelT*     model,
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

          std::vector<ScalarT> elementary_v(n_var);
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

            std::ranges::fill(elementary_v, 0.0);
            elementary_v[var_i] = 1.0;

            __enzyme_fwddiff<void>((void*) EmtModelWrapper<ModelT, ScalarT>::eval,
                                   enzyme_const,
                                   model,
                                   enzyme_const,
                                   y,
                                   enzyme_dup,
                                   yp,
                                   output,
                                   enzyme_const,
                                   u,
                                   enzyme_dupnoneed,
                                   elementary_v.data(),
                                   d_output);
          }

          jac.setValues(scale, rows, cols, vals, nnz);
        }
      };
    } // namespace Sparse
  } // namespace Enzyme
} // namespace GridKit
