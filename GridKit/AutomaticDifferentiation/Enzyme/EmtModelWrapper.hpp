/**
 * @file EmtModelWrapper.hpp
 *
 * @brief Enzyme wrapper for EMT element residuals. The differentiated signature
 * is evaluateInternalResidual(y, yp, u, f), where u is the borrowed input block.
 * No bus or signal scratch arrays.
 *
 */

#pragma once

namespace GridKit
{
  namespace Enzyme
  {
    namespace Sparse
    {
      template <typename ModelT, typename ScalarT>
      struct EmtModelWrapper
      {
        static void eval(ModelT* model, ScalarT* y, ScalarT* yp, ScalarT* u, ScalarT* f)
        {
          model->evaluateInternalResidual(y, yp, u, f);
        }
      };
    } // namespace Sparse
  } // namespace Enzyme
} // namespace GridKit
