#pragma once

#include <variant>

#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/System/SparseAD.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "EMTTestHelpers.hpp"

namespace GridKit
{
  namespace Testing
  {
    template <class ComponentT>
    std::size_t localVariableCount()
    {
      std::size_t count = ComponentT::own_variable_count;
      for (std::size_t port = 0; port < ComponentT::port_count; ++port)
      {
        count += ComponentT::port_width(port);
      }
      return count;
    }

    template <class ComponentT>
    std::size_t localEquationCount()
    {
      std::size_t count = ComponentT::own_equation_count;
      for (std::size_t port = 0; port < ComponentT::port_count; ++port)
      {
        count += ComponentT::port_width(port);
      }
      return count;
    }

    template <class ComponentT>
    bool componentSparseMatchesFiniteDifference(const ComponentT&          component,
                                                const std::vector<double>& local_y,
                                                const std::vector<double>& local_yp,
                                                double                     alpha,
                                                double                     time)
    {
      const std::size_t n = localVariableCount<ComponentT>();
      const std::size_t m = localEquationCount<ComponentT>();

      EMT::System::LocalMap<size_t> map;
      map.variable_indices.resize(n);
      map.equation_indices.resize(m);
      map.equation_accumulates.assign(m, false);
      for (std::size_t idx = 0; idx < n; ++idx)
      {
        map.variable_indices[idx] = idx;
      }
      for (std::size_t idx = 0; idx < m; ++idx)
      {
        map.equation_indices[idx] = idx;
      }

      auto unbound = EMT::System::SparseAD<size_t>::analyze(component, map, time);
      auto csr     = EMT::System::buildCsrPattern<size_t>(m, n, unbound.coordinates);
      auto bound   = EMT::System::SparseAD<size_t>::bind(unbound, csr);

      std::vector<double> values(csr.nnz(), 0.0);
      std::vector<double> work_y;
      std::vector<double> work_yp;
      std::vector<double> work_f;
      EMT::System::SparseAD<size_t>::addComponentJacobian(component, bound, map, local_y, local_yp, alpha, values, time, work_y, work_yp, work_f);

      std::vector<double> base(m, 0.0);
      component.residual(local_y.data(), local_yp.data(), base.data(), time);

      constexpr double eps = 1.0e-7;
      for (std::size_t row = 0; row < m; ++row)
      {
        for (std::size_t slot = csr.row_ptr[row]; slot < csr.row_ptr[row + 1]; ++slot)
        {
          const std::size_t   col     = csr.column_indices[slot];
          auto                pert_y  = local_y;
          auto                pert_yp = local_yp;
          std::vector<double> pert_f(m, 0.0);
          pert_y[col]  += eps;
          pert_yp[col] += alpha * eps;
          component.residual(pert_y.data(), pert_yp.data(), pert_f.data(), time);
          const double fd = (pert_f[row] - base[row]) / eps;
          if (!isEqual(values[slot], fd, 1.0e-5))
          {
            return false;
          }
        }
      }

      return true;
    }

    template <class ScalarT, typename IdxT>
    class EMTComponentModelTests
    {
    private:
      using RealT = typename EMT::Component<ScalarT, IdxT>::RealT;

    public:
      TestOutcome loadRLValidationResidualInitAndSparse()
      {
        TestStatus success = true;

        EMT::LoadRL<ScalarT, IdxT> load(makeLoadRLData<RealT, IdxT>());
        success *= (load.verify() == 0);
        load.allocate();

        std::vector<double> y{1.0, 2.0, 3.0, 10.0, 20.0, 30.0};
        std::vector<double> yp{0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
        std::vector<double> f(6, 0.0);
        load.residual(y.data(), yp.data(), f.data(), 0.0);
        success *= isEqual(f[0], 2.0 * 1.0 + 0.5 * 0.1 + 10.0, 1.0e-12);
        success *= isEqual(f[1], 3.0 * 2.0 + 0.25 * 0.2 + 20.0, 1.0e-12);
        success *= isEqual(f[3], 1.0, 1.0e-12);
        success *= isEqual(f[5], 3.0, 1.0e-12);

        EMT::BusData<RealT, IdxT> bus_data;
        bus_data.name      = "load_bus";
        bus_data.bus_id    = 1;
        bus_data.vm        = 120.0;
        bus_data.va        = 0.0;
        bus_data.freq_base = 60.0;
        EMT::Bus<ScalarT, IdxT> bus(bus_data);
        bus.allocate();
        bus.initialize();
        EMT::LoadRL<ScalarT, IdxT> initialized_load(&bus, makeLoadRLData<RealT, IdxT>(1));
        initialized_load.allocate();
        initialized_load.initialize();

        const auto  voltage      = bus.initialVoltagePhasor();
        const RealT omega        = 2.0 * std::acos(RealT{-1.0}) * 60.0;
        const auto  expected_i0  = -voltage[0] / std::complex<RealT>{2.0, omega * 0.5};
        success                 *= isEqual(static_cast<RealT>(initialized_load.y()[0]),
                           std::sqrt(RealT{2.0}) * std::real(expected_i0),
                           1.0e-12);

        success *= componentSparseMatchesFiniteDifference(load, y, yp, 5.0, 0.0);

        auto bad                                 = makeLoadRLData<RealT, IdxT>();
        bad.parameters[EMT::LoadRLParameters::l] = EMT::PhaseVector<RealT>{0.0, 1.0, 1.0};
        EMT::LoadRL<ScalarT, IdxT> bad_load(bad);
        success *= (bad_load.verify() > 0);

        return success.report(__func__);
      }

      TestOutcome voltageSourceValidationResidualAndSparse()
      {
        TestStatus success = true;

        EMT::VoltageSource<ScalarT, IdxT> source(makeVoltageSourceData<RealT, IdxT>());
        success *= (source.verify() == 0);
        success *= isEqual(source.omega0(), 2.0 * std::acos(RealT{-1.0}) * 60.0, 1.0e-12);

        const double        time = 0.017;
        std::vector<double> y{10.0, -20.0, 30.0};
        std::vector<double> yp(3, 0.0);
        std::vector<double> f(3, 0.0);
        source.residual(y.data(), yp.data(), f.data(), time);
        const auto&  e   = source.e();
        const auto&  phi = source.phi();
        const auto&  r   = source.r();
        const double expected =
            (std::sqrt(2.0) * e[0] * std::cos(source.omega0() * time + phi[0]) - y[0]) / r[0];
        success *= isEqual(f[0], expected, 1.0e-12);
        success *= componentSparseMatchesFiniteDifference(source, y, yp, 3.0, time);

        auto bad                                             = makeVoltageSourceData<RealT, IdxT>();
        bad.parameters[EMT::VoltageSourceParameters::omega0] = RealT{1.0};
        EMT::VoltageSource<ScalarT, IdxT> bad_source(bad);
        success *= (bad_source.verify() > 0);

        return success.report(__func__);
      }

      TestOutcome branchValidationResidualInitAndSparse()
      {
        TestStatus success = true;

        EMT::BranchLumpedConstant<ScalarT, IdxT> branch(makeBranchData<RealT, IdxT>());
        success *= (branch.verify() == 0);
        branch.allocate();

        std::vector<double> y{1.0, 2.0, 3.0, 10.0, 20.0, 30.0, -5.0, -4.0, -3.0};
        std::vector<double> yp{0.1, 0.2, 0.3, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0};
        std::vector<double> f(9, 0.0);
        branch.residual(y.data(), yp.data(), f.data(), 0.0);

        double expected_eq0 = -5.0 - 10.0;
        for (std::size_t col = 0; col < 3; ++col)
        {
          expected_eq0 += branch.R()[0][col] * y[col] + branch.L()[0][col] * yp[col];
        }
        double expected_if0 = -y[0];
        double expected_it0 = y[0];
        for (std::size_t col = 0; col < 3; ++col)
        {
          expected_if0 -= branch.GHalf()[0][col] * y[3 + col]
                          + branch.CHalf()[0][col] * yp[3 + col];
          expected_it0 -= branch.GHalf()[0][col] * y[6 + col]
                          + branch.CHalf()[0][col] * yp[6 + col];
        }
        success *= isEqual(f[0], expected_eq0, 1.0e-12);
        success *= isEqual(f[3], expected_if0, 1.0e-12);
        success *= isEqual(f[6], expected_it0, 1.0e-12);
        success *= componentSparseMatchesFiniteDifference(branch, y, yp, 2.0, 0.0);

        EMT::BusData<RealT, IdxT> from_bus_data;
        from_bus_data.name      = "from_bus";
        from_bus_data.bus_id    = 0;
        from_bus_data.vm        = 120.0;
        from_bus_data.va        = 0.0;
        from_bus_data.freq_base = 60.0;
        EMT::BusData<RealT, IdxT> to_bus_data;
        to_bus_data.name      = "to_bus";
        to_bus_data.bus_id    = 1;
        to_bus_data.vm        = 118.0;
        to_bus_data.va        = -0.05;
        to_bus_data.freq_base = 60.0;
        EMT::Bus<ScalarT, IdxT> from_bus(from_bus_data);
        EMT::Bus<ScalarT, IdxT> to_bus(to_bus_data);
        from_bus.allocate();
        to_bus.allocate();
        from_bus.initialize();
        to_bus.initialize();
        EMT::BranchLumpedConstant<ScalarT, IdxT> initialized_branch(
            &from_bus, &to_bus, makeBranchData<RealT, IdxT>(0, 1));
        initialized_branch.allocate();
        initialized_branch.initialize();

        const auto                            from_v = from_bus.initialVoltagePhasor();
        const auto                            to_v   = to_bus.initialVoltagePhasor();
        const RealT                           omega  = 2.0 * std::acos(RealT{-1.0}) * 60.0;
        EMT::PhaseMatrix<std::complex<RealT>> z{};
        for (std::size_t row = 0; row < 3; ++row)
        {
          for (std::size_t col = 0; col < 3; ++col)
          {
            z[row][col] = {initialized_branch.R()[row][col], omega * initialized_branch.L()[row][col]};
          }
        }
        EMT::PhaseVector<std::complex<RealT>> drop{};
        for (std::size_t phase = 0; phase < 3; ++phase)
        {
          drop[phase] = from_v[phase] - to_v[phase];
        }
        const auto expected_i  = EMT::solve(z, drop);
        success               *= isEqual(static_cast<RealT>(initialized_branch.y()[1]),
                           std::sqrt(RealT{2.0}) * std::real(expected_i[1]),
                           1.0e-11);

        auto bad                                               = makeBranchData<RealT, IdxT>();
        auto c                                                 = std::get<EMT::PhaseMatrix<RealT>>(bad.parameters[EMT::BranchLumpedConstantParameters::c]);
        c[0][0]                                                = 0.0;
        bad.parameters[EMT::BranchLumpedConstantParameters::c] = c;
        EMT::BranchLumpedConstant<ScalarT, IdxT> bad_branch(bad);
        success *= (bad_branch.verify() > 0);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
