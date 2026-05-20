#pragma once

#include <GridKit/Model/EMT/System/SparseAD.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    struct FakeAffineEMTModel
    {
      static constexpr std::size_t own_variable_count = 1;
      static constexpr std::size_t own_equation_count = 1;
      static constexpr std::size_t port_count         = 1;
      static constexpr bool        is_affine          = true;

      static constexpr std::size_t port_width(std::size_t)
      {
        return 3;
      }

      static constexpr bool own_differential(std::size_t)
      {
        return true;
      }

      template <class S>
      void residual(const S* y, const S* yp, S* f, double) const
      {
        f[0] = 2.0 * y[0] + 3.0 * yp[0] + y[1] + 4.0 * yp[2];
        f[1] = 5.0 * y[0] - y[1];
        f[2] = yp[3];
        f[3] = y[0] + yp[0] + y[3];
      }
    };

    struct FakeNonlinearEMTModel
    {
      static constexpr std::size_t own_variable_count = 2;
      static constexpr std::size_t own_equation_count = 1;
      static constexpr std::size_t port_count         = 0;
      static constexpr bool        is_affine          = false;

      static constexpr std::size_t port_width(std::size_t)
      {
        return 0;
      }

      static constexpr bool own_differential(std::size_t)
      {
        return true;
      }

      template <class S>
      void residual(const S* y, const S* yp, S* f, double) const
      {
        f[0] = y[0] * y[1] + yp[0] * yp[1];
      }
    };

    class EMTSparseADTests
    {
    public:
      TestOutcome affinePatternTracksYAndYpSeparately()
      {
        TestStatus success = true;

        EMT::System::LocalMap<size_t> map;
        map.variable_indices     = {3, 0, 1, 2};
        map.equation_indices     = {3, 0, 1, 2};
        map.equation_accumulates = {false, true, true, true};

        FakeAffineEMTModel model;
        auto               unbound = EMT::System::SparseAD<size_t>::analyze(model, map, 0.0);
        auto               csr     = EMT::System::buildCsrPattern<size_t>(4, 4, unbound.coordinates);
        auto               bound   = EMT::System::SparseAD<size_t>::bind(unbound, csr);

        std::vector<double> values(csr.nnz(), 0.0);
        std::vector<double> y{10.0, 20.0, 30.0, 40.0};
        std::vector<double> yp{1.0, 2.0, 3.0, 4.0};
        std::vector<double> local_y;
        std::vector<double> local_yp;
        std::vector<double> work;
        const double        alpha = 7.0;
        EMT::System::SparseAD<size_t>::addComponentJacobian(model, bound, map, y, yp, alpha, values, 0.0, local_y, local_yp, work);

        success *= isEqual(values[csr.slot(3, 3)], 2.0 + alpha * 3.0, 1.0e-12);
        success *= isEqual(values[csr.slot(3, 1)], alpha * 4.0, 1.0e-12);
        success *= (unbound.derivative_columns.contains(1));
        success *= (unbound.derivative_columns.contains(3));

        return success.report(__func__);
      }

      TestOutcome nonlinearValuePathMatchesFiniteDifference()
      {
        TestStatus success = true;

        EMT::System::LocalMap<size_t> map;
        map.variable_indices     = {0, 1};
        map.equation_indices     = {0};
        map.equation_accumulates = {false};

        FakeNonlinearEMTModel model;
        auto                  unbound = EMT::System::SparseAD<size_t>::analyze(model, map, 0.0);
        auto                  csr     = EMT::System::buildCsrPattern<size_t>(1, 2, unbound.coordinates);
        auto                  bound   = EMT::System::SparseAD<size_t>::bind(unbound, csr);

        std::vector<double> y{2.0, 5.0};
        std::vector<double> yp{3.0, 7.0};
        std::vector<double> values(csr.nnz(), 0.0);
        std::vector<double> local_y;
        std::vector<double> local_yp;
        std::vector<double> work;
        const double        alpha = 11.0;
        EMT::System::SparseAD<size_t>::addComponentJacobian(model, bound, map, y, yp, alpha, values, 0.0, local_y, local_yp, work);

        success *= isEqual(values[csr.slot(0, 0)], y[1] + alpha * yp[1], 1.0e-9);
        success *= isEqual(values[csr.slot(0, 1)], y[0] + alpha * yp[0], 1.0e-9);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
