#include "BusImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::evaluateJacobian()
    {
      if (J_.nnz() == 0)
      {
        std::vector<IdxT>  rows;
        std::vector<IdxT>  cols;
        std::vector<RealT> vals;

        rows.reserve(static_cast<std::size_t>(size_) * static_cast<std::size_t>(size_));
        cols.reserve(rows.capacity());
        vals.reserve(rows.capacity());

        for (IdxT r = 0; r < size_; ++r)
        {
          for (IdxT c = 0; c < size_; ++c)
          {
            rows.push_back(residual_indices_[static_cast<std::size_t>(r)]);
            cols.push_back(variable_indices_[static_cast<std::size_t>(c)]);
            vals.push_back(0.0);
          }
        }

        J_.setValues(rows, cols, vals);
      }
      else
      {
        J_.zeroValuedMatrix();
      }

      return 0;
    }

    template class Bus<double, long int>;
    template class Bus<double, size_t>;
  } // namespace EMT
} // namespace GridKit
