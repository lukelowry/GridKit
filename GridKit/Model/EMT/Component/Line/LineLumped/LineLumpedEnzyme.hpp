#pragma once

#include <tuple>
#include <utility>
#include <vector>

#ifdef GRIDKIT_ENABLE_ENZYME
namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N>
    int LineLumped<scalar_type, index_type, N>::evaluateJacobian()
    {
      copyParentIndicesToSubmodels();

      int ret  = 0;
      ret     += series_z_.evaluateJacobian();
      ret     += shunt_y1_.evaluateJacobian();
      ret     += shunt_y2_.evaluateJacobian();

      J_.zeroMatrix();

      std::vector<IdxT>  rows;
      std::vector<IdxT>  cols;
      std::vector<RealT> vals;

      auto appendSubmodelJacobian = [&](VectorFitT& model)
      {
        auto  entries    = model.getJacobian().getEntries(false);
        auto& child_rows = std::get<0>(entries);
        auto& child_cols = std::get<1>(entries);
        auto& child_vals = std::get<2>(entries);

        rows.insert(rows.end(), child_rows.begin(), child_rows.end());
        cols.insert(cols.end(), child_cols.begin(), child_cols.end());
        vals.insert(vals.end(), child_vals.begin(), child_vals.end());
      };

      appendSubmodelJacobian(series_z_);
      appendSubmodelJacobian(shunt_y1_);
      appendSubmodelJacobian(shunt_y2_);

      auto store = [&](IdxT row, IdxT col, RealT val)
      {
        if (val == RealT{0.0})
        {
          return;
        }

        rows.push_back(row);
        cols.push_back(col);
        vals.push_back(val);
      };

      for (std::size_t n = 0; n < N; ++n)
      {
        const auto idx = static_cast<IdxT>(n);

        store(residual_indices_[n], variable_indices_[series_z_output_ + n], RealT{1.0});
        store(residual_indices_[n], bus2_->getVariableIndex(idx), RealT{1.0});
        store(residual_indices_[n], bus1_->getVariableIndex(idx), RealT{-1.0});

        store(bus1_->getResidualIndex(idx), variable_indices_[n], RealT{-1.0});
        store(bus1_->getResidualIndex(idx), variable_indices_[shunt_y1_output_ + n], RealT{-1.0});
        store(bus2_->getResidualIndex(idx), variable_indices_[n], RealT{1.0});
        store(bus2_->getResidualIndex(idx), variable_indices_[shunt_y2_output_ + n], RealT{-1.0});
      }

      J_.setValues(std::move(rows), std::move(cols), std::move(vals));
      return ret;
    }
  } // namespace EMT
} // namespace GridKit
#endif
