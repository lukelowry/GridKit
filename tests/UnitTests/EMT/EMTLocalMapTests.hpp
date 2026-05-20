#pragma once

#include <GridKit/Model/EMT/System/CsrAssembly.hpp>
#include <GridKit/Model/EMT/System/LocalMap.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    class EMTLocalMapTests
    {
    public:
      TestOutcome gatherScatterAndAccumulate()
      {
        TestStatus success = true;

        EMT::System::LocalMap<size_t> map;
        map.variable_indices     = {3, 0, 1, 2};
        map.equation_indices     = {3, 0, 1, 2};
        map.equation_accumulates = {false, true, true, true};
        map.own_variable_count   = 1;
        map.own_equation_count   = 1;

        std::vector<double> y{10.0, 20.0, 30.0, 40.0};
        std::vector<double> yp{1.0, 2.0, 3.0, 4.0};
        std::vector<double> local_y;
        std::vector<double> local_yp;
        map.gather(y, yp, local_y, local_yp);

        success *= isEqual(local_y[0], 40.0);
        success *= isEqual(local_y[1], 10.0);
        success *= isEqual(local_yp[3], 3.0);

        std::vector<double> f{100.0, 200.0, 300.0, 400.0};
        map.scatterResidual(std::vector<double>{7.0, 1.0, 2.0, 3.0}, f);
        success *= isEqual(f[3], 7.0);
        success *= isEqual(f[0], 101.0);
        success *= isEqual(f[1], 202.0);
        success *= isEqual(f[2], 303.0);

        return success.report(__func__);
      }

      TestOutcome csrDeduplicatesStableSlots()
      {
        TestStatus success = true;

        std::vector<std::pair<size_t, size_t>> coordinates{
            {0, 0}, {0, 1}, {0, 0}, {2, 3}, {1, 1}, {2, 3}};
        auto csr = EMT::System::buildCsrPattern<size_t>(3, 4, coordinates);

        success *= (csr.nnz() == 4);
        success *= (csr.row_ptr == std::vector<size_t>{0, 2, 3, 4});
        success *= (csr.column_indices == std::vector<size_t>{0, 1, 1, 3});
        success *= (csr.slot(0, 0) == 0);
        success *= (csr.slot(2, 3) == 3);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
