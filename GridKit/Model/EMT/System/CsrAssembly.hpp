#pragma once

#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace GridKit
{
  namespace EMT
  {
    namespace System
    {
      template <typename IdxT>
      struct CsrPattern
      {
        IdxT                                  row_count{0};
        IdxT                                  column_count{0};
        std::vector<IdxT>                     row_ptr;
        std::vector<IdxT>                     column_indices;
        std::map<std::pair<IdxT, IdxT>, IdxT> slot_by_coordinate;

        IdxT nnz() const
        {
          return static_cast<IdxT>(column_indices.size());
        }

        IdxT slot(IdxT row, IdxT column) const
        {
          const auto iter = slot_by_coordinate.find({row, column});
          if (iter == slot_by_coordinate.end())
          {
            throw std::out_of_range("EMT CSR coordinate was not bound in the sparse pattern");
          }
          return iter->second;
        }
      };

      template <typename IdxT>
      CsrPattern<IdxT> buildCsrPattern(IdxT                               row_count,
                                       IdxT                               column_count,
                                       std::vector<std::pair<IdxT, IdxT>> coordinates)
      {
        coordinates.erase(
            std::remove_if(coordinates.begin(),
                           coordinates.end(),
                           [row_count, column_count](const auto& coordinate)
                           {
                             return coordinate.first < IdxT{0}
                                    || coordinate.second < IdxT{0}
                                    || coordinate.first >= row_count
                                    || coordinate.second >= column_count;
                           }),
            coordinates.end());

        std::sort(coordinates.begin(), coordinates.end());
        coordinates.erase(std::unique(coordinates.begin(), coordinates.end()), coordinates.end());

        CsrPattern<IdxT> pattern;
        pattern.row_count    = row_count;
        pattern.column_count = column_count;
        pattern.row_ptr.assign(static_cast<std::size_t>(row_count + IdxT{1}), IdxT{0});
        pattern.column_indices.reserve(coordinates.size());

        for (const auto& [row, column] : coordinates)
        {
          ++pattern.row_ptr[static_cast<std::size_t>(row + IdxT{1})];
          pattern.column_indices.push_back(column);
        }

        for (std::size_t row = 0; row + 1 < pattern.row_ptr.size(); ++row)
        {
          pattern.row_ptr[row + 1] += pattern.row_ptr[row];
        }

        for (IdxT slot = 0; slot < static_cast<IdxT>(coordinates.size()); ++slot)
        {
          pattern.slot_by_coordinate[coordinates[static_cast<std::size_t>(slot)]] = slot;
        }

        return pattern;
      }
    } // namespace System
  } // namespace EMT
} // namespace GridKit
