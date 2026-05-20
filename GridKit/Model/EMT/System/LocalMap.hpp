#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace GridKit
{
  namespace EMT
  {
    namespace System
    {
      template <typename IdxT>
      struct LocalMap
      {
        std::vector<IdxT> variable_indices;
        std::vector<IdxT> equation_indices;
        std::vector<bool> equation_accumulates;

        std::size_t own_variable_count{0};
        std::size_t own_equation_count{0};

        std::size_t localVariableCount() const
        {
          return variable_indices.size();
        }

        std::size_t localEquationCount() const
        {
          return equation_indices.size();
        }

        template <class ScalarT>
        void gather(const std::vector<ScalarT>& global_y,
                    const std::vector<ScalarT>& global_yp,
                    std::vector<ScalarT>&       local_y,
                    std::vector<ScalarT>&       local_yp) const
        {
          local_y.resize(variable_indices.size());
          local_yp.resize(variable_indices.size());
          for (std::size_t local = 0; local < variable_indices.size(); ++local)
          {
            const auto global = static_cast<std::size_t>(variable_indices[local]);
            local_y[local]    = global_y.at(global);
            local_yp[local]   = global_yp.at(global);
          }
        }

        template <class ScalarT>
        void scatterResidual(const std::vector<ScalarT>& local_f,
                             std::vector<ScalarT>&       global_f) const
        {
          if (local_f.size() != equation_indices.size())
          {
            throw std::invalid_argument("EMT local residual size does not match local equation map");
          }

          for (std::size_t local = 0; local < equation_indices.size(); ++local)
          {
            const auto global = static_cast<std::size_t>(equation_indices[local]);
            if (equation_accumulates.at(local))
            {
              global_f.at(global) += local_f[local];
            }
            else
            {
              global_f.at(global) = local_f[local];
            }
          }
        }
      };
    } // namespace System
  } // namespace EMT
} // namespace GridKit
