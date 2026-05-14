#pragma once

#include <stdexcept>

namespace GridKit
{
  namespace EMT
  {
    template <class ModelT>
    struct EnzymeJacobian
    {
      template <class PatternT>
      static void pattern(const ModelT&, PatternT&)
      {
        throw std::logic_error("EMT Enzyme Jacobian pattern specialization is not available for this model");
      }

      template <class StateT, class JacobianT>
      static void evaluate(const ModelT&, const StateT&, JacobianT&)
      {
        throw std::logic_error("EMT Enzyme Jacobian specialization is not available for this model");
      }
    };
  } // namespace EMT
} // namespace GridKit
