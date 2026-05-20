#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/AutomaticDifferentiation/Enzyme/EnzymeDefinitions.hpp>
#include <GridKit/Model/EMT/System/CsrAssembly.hpp>
#include <GridKit/Model/EMT/System/LocalMap.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace System
    {
      template <typename IdxT>
      struct UnboundAffineEntry
      {
        IdxT   row{0};
        IdxT   column{0};
        double coefficient_y{0.0};
        double coefficient_yp{0.0};
      };

      template <typename IdxT>
      struct UnboundNonlinearEntry
      {
        IdxT        row{0};
        IdxT        column{0};
        std::size_t local_row{0};
        std::size_t local_column{0};
        bool        derivative_column{false};
      };

      template <typename IdxT>
      struct UnboundComponentJacobian
      {
        bool                                     affine{true};
        std::vector<UnboundAffineEntry<IdxT>>    affine_entries;
        std::vector<UnboundNonlinearEntry<IdxT>> nonlinear_entries;
        std::vector<std::pair<IdxT, IdxT>>       coordinates;
        std::set<IdxT>                           derivative_columns;
      };

      template <typename IdxT>
      struct BoundAffineEntry
      {
        IdxT   csr_slot{0};
        double coefficient_y{0.0};
        double coefficient_yp{0.0};
      };

      template <typename IdxT>
      struct BoundNonlinearEntry
      {
        IdxT        csr_slot{0};
        std::size_t local_row{0};
        std::size_t local_column{0};
        bool        derivative_column{false};
      };

      template <typename IdxT>
      struct ComponentJacobian
      {
        bool                                   affine{true};
        std::vector<BoundAffineEntry<IdxT>>    affine_entries;
        std::vector<BoundNonlinearEntry<IdxT>> nonlinear_entries;
      };

      template <class ComponentT>
      __attribute__((always_inline)) inline void enzymeResidualKernel(ComponentT* component,
                                                                      double*     y,
                                                                      double*     yp,
                                                                      double*     f,
                                                                      double      t)
      {
        component->residual(y, yp, f, t);
      }

      template <typename IdxT>
      class SparseAD
      {
      public:
        template <class ComponentT>
        static UnboundComponentJacobian<IdxT> analyze(const ComponentT&     component,
                                                      const LocalMap<IdxT>& map,
                                                      double                time)
        {
          using ADScalar = DependencyTracking::Variable;

          const std::size_t n_local_variables = map.localVariableCount();
          const std::size_t n_local_equations = map.localEquationCount();

          std::vector<ADScalar> local_y(n_local_variables);
          std::vector<ADScalar> local_yp(n_local_variables);
          std::vector<ADScalar> local_f(n_local_equations);

          for (std::size_t idx = 0; idx < n_local_variables; ++idx)
          {
            local_y[idx]  = ADScalar(1.0 + 0.01 * static_cast<double>(idx), idx);
            local_yp[idx] = ADScalar(2.0 + 0.01 * static_cast<double>(idx),
                                     n_local_variables + idx);
          }

          component.residual(local_y.data(), local_yp.data(), local_f.data(), time);

          UnboundComponentJacobian<IdxT> result;
          result.affine = ComponentT::is_affine;

          std::map<std::pair<IdxT, IdxT>, std::pair<double, double>> affine_coefficients;

          for (std::size_t local_row = 0; local_row < n_local_equations; ++local_row)
          {
            const IdxT global_row = map.equation_indices.at(local_row);
            for (const auto& [dependency, coefficient] : local_f[local_row].getDependencies())
            {
              if (coefficient == 0.0)
              {
                continue;
              }

              const bool        is_yp        = dependency >= n_local_variables;
              const std::size_t local_column = is_yp ? dependency - n_local_variables : dependency;
              const IdxT        global_col   = map.variable_indices.at(local_column);

              result.coordinates.push_back({global_row, global_col});
              if (is_yp)
              {
                result.derivative_columns.insert(global_col);
              }

              if (result.affine)
              {
                auto& coefficients = affine_coefficients[{global_row, global_col}];
                if (is_yp)
                {
                  coefficients.second += coefficient;
                }
                else
                {
                  coefficients.first += coefficient;
                }
              }
              else
              {
                result.nonlinear_entries.push_back(
                    {global_row, global_col, local_row, local_column, is_yp});
              }
            }
          }

          if (result.affine)
          {
            result.affine_entries.reserve(affine_coefficients.size());
            for (const auto& [coordinate, coefficients] : affine_coefficients)
            {
              result.affine_entries.push_back(
                  {coordinate.first, coordinate.second, coefficients.first, coefficients.second});
            }
          }

          return result;
        }

        static ComponentJacobian<IdxT> bind(const UnboundComponentJacobian<IdxT>& unbound,
                                            const CsrPattern<IdxT>&               csr)
        {
          ComponentJacobian<IdxT> bound;
          bound.affine = unbound.affine;

          if (unbound.affine)
          {
            bound.affine_entries.reserve(unbound.affine_entries.size());
            for (const auto& entry : unbound.affine_entries)
            {
              bound.affine_entries.push_back(
                  {csr.slot(entry.row, entry.column), entry.coefficient_y, entry.coefficient_yp});
            }
          }
          else
          {
            bound.nonlinear_entries.reserve(unbound.nonlinear_entries.size());
            for (const auto& entry : unbound.nonlinear_entries)
            {
              bound.nonlinear_entries.push_back(
                  {csr.slot(entry.row, entry.column), entry.local_row, entry.local_column, entry.derivative_column});
            }
          }

          return bound;
        }

        template <class ComponentT>
        static void addComponentJacobian(const ComponentT&              component,
                                         const ComponentJacobian<IdxT>& jacobian,
                                         const LocalMap<IdxT>&          map,
                                         const std::vector<double>&     global_y,
                                         const std::vector<double>&     global_yp,
                                         double                         alpha,
                                         std::vector<double>&           csr_values,
                                         double                         time,
                                         std::vector<double>&           local_y,
                                         std::vector<double>&           local_yp,
                                         std::vector<double>&           work_f)
        {
          if (jacobian.affine)
          {
            for (const auto& entry : jacobian.affine_entries)
            {
              csr_values.at(static_cast<std::size_t>(entry.csr_slot)) +=
                  entry.coefficient_y + alpha * entry.coefficient_yp;
            }
            return;
          }

          map.gather(global_y, global_yp, local_y, local_yp);
          const std::size_t n_local_variables = map.localVariableCount();
          const std::size_t n_local_equations = map.localEquationCount();

          std::vector<std::size_t> y_columns;
          std::vector<std::size_t> yp_columns;
          for (const auto& entry : jacobian.nonlinear_entries)
          {
            auto& columns = entry.derivative_column ? yp_columns : y_columns;
            if (std::find(columns.begin(), columns.end(), entry.local_column) == columns.end())
            {
              columns.push_back(entry.local_column);
            }
          }

          std::vector<double> seed_y(n_local_variables, 0.0);
          std::vector<double> seed_yp(n_local_variables, 0.0);
          std::vector<double> derivative_f(n_local_equations, 0.0);
          work_f.assign(n_local_equations, 0.0);

          auto evaluate_column =
              [&](std::size_t active_column, bool derivative_column, double scale)
          {
            std::fill(seed_y.begin(), seed_y.end(), 0.0);
            std::fill(seed_yp.begin(), seed_yp.end(), 0.0);
            std::fill(derivative_f.begin(), derivative_f.end(), 0.0);
            std::fill(work_f.begin(), work_f.end(), 0.0);

            if (derivative_column)
            {
              seed_yp[active_column] = 1.0;
            }
            else
            {
              seed_y[active_column] = 1.0;
            }

            using namespace GridKit::Enzyme::Sparse;
            __enzyme_fwddiff<void>((void*) enzymeResidualKernel<ComponentT>,
                                   enzyme_const,
                                   const_cast<ComponentT*>(&component),
                                   enzyme_dup,
                                   local_y.data(),
                                   seed_y.data(),
                                   enzyme_dup,
                                   local_yp.data(),
                                   seed_yp.data(),
                                   enzyme_dupnoneed,
                                   work_f.data(),
                                   derivative_f.data(),
                                   enzyme_const,
                                   time);

            for (const auto& entry : jacobian.nonlinear_entries)
            {
              if (entry.derivative_column == derivative_column
                  && entry.local_column == active_column)
              {
                csr_values.at(static_cast<std::size_t>(entry.csr_slot)) +=
                    scale * derivative_f.at(entry.local_row);
              }
            }
          };

          for (const auto column : y_columns)
          {
            evaluate_column(column, false, 1.0);
          }
          for (const auto column : yp_columns)
          {
            evaluate_column(column, true, alpha);
          }
        }
      };
    } // namespace System
  } // namespace EMT
} // namespace GridKit
