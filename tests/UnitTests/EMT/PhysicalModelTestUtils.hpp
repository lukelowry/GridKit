#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>

#include "EMTTestFixture.hpp"

namespace GridKit::Testing::EMTTest
{
  using DenseJacobian = std::vector<std::vector<RealT>>;

  inline DenseJacobian centeredDifferenceJacobian(SystemT& system,
                                                  RealT    time,
                                                  RealT    alpha)
  {
    const auto    size = system.size();
    DenseJacobian jacobian(size, std::vector<RealT>(size, 0.0));

    auto*              y  = system.y().getData();
    auto*              yp = system.yp().getData();
    std::vector<RealT> y0(y, y + size);
    std::vector<RealT> yp0(yp, yp + size);
    std::vector<RealT> plus(size);
    std::vector<RealT> minus(size);

    system.updateTime(time, alpha);
    for (IdxT column = 0; column < size; ++column)
    {
      const RealT step = 1.0e-5 * (1.0 + std::abs(y0[column]));

      y[column]  = y0[column] + step;
      yp[column] = yp0[column] + alpha * step;
      system.y().setDataUpdated();
      system.yp().setDataUpdated();
      system.evaluateResidual();
      std::copy(system.getResidual().getData(),
                system.getResidual().getData() + size,
                plus.begin());

      y[column]  = y0[column] - step;
      yp[column] = yp0[column] - alpha * step;
      system.y().setDataUpdated();
      system.yp().setDataUpdated();
      system.evaluateResidual();
      std::copy(system.getResidual().getData(),
                system.getResidual().getData() + size,
                minus.begin());

      y[column]  = y0[column];
      yp[column] = yp0[column];
      for (IdxT row = 0; row < size; ++row)
      {
        jacobian[row][column] = (plus[row] - minus[row]) / (2.0 * step);
      }
    }

    system.y().setDataUpdated();
    system.yp().setDataUpdated();
    system.evaluateResidual();
    return jacobian;
  }

  inline DenseJacobian dependencyTrackingJacobian(const DataT& data,
                                                  RealT        time,
                                                  RealT        alpha)
  {
    using DepVar    = DependencyTracking::Variable;
    using DepSystem = EMT::SystemModel<DepVar, IdxT>;

    DepSystem system(data);
    system.updateTime(time, alpha);
    system.allocate();
    system.tagDifferentiable();
    system.setAbsoluteTolerance(1.0e-8);
    system.initialize();

    const auto         size = system.size();
    DenseJacobian      jacobian(size, std::vector<RealT>(size, 0.0));
    auto*              y  = system.y().getData();
    auto*              yp = system.yp().getData();
    std::vector<RealT> y0(size);
    std::vector<RealT> yp0(size);

    for (IdxT column = 0; column < size; ++column)
    {
      y0[column]  = y[column].getValue();
      yp0[column] = yp[column].getValue();
      y[column]   = y0[column];
      yp[column]  = yp0[column];
      y[column].setVariableNumber(column);
    }
    system.y().setDataUpdated();
    system.yp().setDataUpdated();
    system.evaluateResidual();

    for (IdxT row = 0; row < size; ++row)
    {
      for (const auto& [column, value] : system.getResidual().getData()[row].getDependencies())
      {
        if (column < size)
        {
          jacobian[row][column] += value;
        }
      }
    }

    for (IdxT column = 0; column < size; ++column)
    {
      y[column]  = y0[column];
      yp[column] = yp0[column];
      yp[column].setVariableNumber(column);
    }
    system.y().setDataUpdated();
    system.yp().setDataUpdated();
    system.evaluateResidual();

    for (IdxT row = 0; row < size; ++row)
    {
      for (const auto& [column, value] : system.getResidual().getData()[row].getDependencies())
      {
        if (column < size)
        {
          jacobian[row][column] += alpha * value;
        }
      }
    }
    return jacobian;
  }

  inline bool closeJacobianValue(RealT value, RealT reference, RealT tolerance)
  {
    return std::abs(value - reference)
           <= tolerance * (1.0 + std::max(std::abs(value), std::abs(reference)));
  }

  template <typename ComponentT>
  bool componentJacobianMatchesReferences(SystemT&                  system,
                                          ComponentT&               component,
                                          const DataT&              data,
                                          const std::vector<RealT>& alphas,
                                          bool                      component_owns_rows = true)
  {
#ifndef GRIDKIT_ENABLE_ENZYME
    static_cast<void>(system);
    static_cast<void>(component);
    static_cast<void>(data);
    static_cast<void>(alphas);
    static_cast<void>(component_owns_rows);
    return true;
#else
    const RealT          time             = 0.013;
    const auto&          residual_indices = component.getResidualIndices();
    const auto&          variable_indices = component.getVariableIndices();
    const std::set<IdxT> owned_rows(residual_indices.begin(), residual_indices.end());
    const std::set<IdxT> owned_columns(variable_indices.begin(), variable_indices.end());

    for (const RealT alpha : alphas)
    {
      system.updateTime(time, alpha);
      if (system.evaluateResidual() != 0 || system.evaluateJacobian() != 0)
      {
        return false;
      }

      std::set<std::pair<IdxT, IdxT>> entries;
      if (component_owns_rows)
      {
        for (const IdxT row : residual_indices)
        {
          for (IdxT column = 0; column < system.size(); ++column)
          {
            entries.emplace(row, column);
          }
        }
      }
      else
      {
        for (const IdxT row : residual_indices)
        {
          for (const IdxT column : variable_indices)
          {
            entries.emplace(row, column);
          }
        }
      }

      auto* coo = component.getCooJacobian();
      if (coo == nullptr)
      {
        return false;
      }
      for (IdxT entry = 0; entry < coo->getNnz(); ++entry)
      {
        const IdxT row    = coo->getRowData()[entry];
        const IdxT column = coo->getColData()[entry];
        if (!owned_rows.contains(row) && owned_columns.contains(column))
        {
          entries.emplace(row, column);
        }
      }

      const auto finite_difference = centeredDifferenceJacobian(system, time, alpha);
      const auto dependency        = dependencyTrackingJacobian(data, time, alpha);
      if (component_owns_rows)
      {
        for (IdxT row = 0; row < system.size(); ++row)
        {
          if (owned_rows.contains(row))
          {
            continue;
          }
          for (const IdxT column : variable_indices)
          {
            if (!closeJacobianValue(finite_difference[row][column], 0.0, 5.0e-7)
                || !closeJacobianValue(dependency[row][column], 0.0, 2.0e-12))
            {
              entries.emplace(row, column);
            }
          }
        }
      }
      for (const auto& [row, column] : entries)
      {
        const RealT enzyme_value = cooValue(component, row, column);
        if (!closeJacobianValue(enzyme_value,
                                finite_difference[row][column],
                                5.0e-7))
        {
          std::cerr << "Centered-difference mismatch at alpha=" << alpha
                    << ", row=" << row << ", column=" << column
                    << ": Enzyme=" << enzyme_value
                    << ", finite difference=" << finite_difference[row][column]
                    << '\n';
          return false;
        }
        if (!closeJacobianValue(enzyme_value, dependency[row][column], 2.0e-12))
        {
          std::cerr << "Dependency-tracking mismatch at alpha=" << alpha
                    << ", row=" << row << ", column=" << column
                    << ": Enzyme=" << enzyme_value
                    << ", dependency tracking=" << dependency[row][column]
                    << '\n';
          return false;
        }
      }
    }
    return true;
#endif
  }
} // namespace GridKit::Testing::EMTTest
