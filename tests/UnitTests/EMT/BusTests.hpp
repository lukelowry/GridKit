#pragma once

#include <array>
#include <tuple>
#include <vector>

#include <GridKit/LinearAlgebra/SparseMatrix/COO_Matrix.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/Errors.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class CurrentInjectionComponent final : public EMT::Component<ScalarT, IdxT>
    {
    public:
      using BaseT = EMT::Component<ScalarT, IdxT>;
      using RealT = typename BaseT::RealT;

      void apply(EMT::Bus<ScalarT, IdxT>& bus, const ScalarT* i_inj)
      {
        this->addCurrentInjection(bus, i_inj);
      }

      int verify() const override
      {
        return 0;
      }

      int allocate() override
      {
        return 0;
      }

      int initialize() override
      {
        return 0;
      }

      int tagDifferentiable() override
      {
        return 0;
      }

      int setAbsoluteTolerance(RealT) override
      {
        return 0;
      }

      int evaluateResidual() override
      {
        return 0;
      }

      int evaluateJacobian() override
      {
        return 0;
      }

      int setGridKitComponentID(IdxT gridkit_component_id) override
      {
        this->gridkit_component_id_ = gridkit_component_id;
        return 0;
      }
    };

    template <class ScalarT, typename IdxT>
    class BusTests
    {
    public:
      using RealT = typename EMT::Bus<ScalarT, IdxT>::RealT;

      BusTests()  = default;
      ~BusTests() = default;

      TestOutcome defaultConstruction()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus;

        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;

        success *= bus.phaseCount() == 3;
        success *= bus.size() == 3;
        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          success *= isEqual(bus.V(n), ScalarT{0.0});
          success *= isEqual(bus.Vp(n), ScalarT{0.0});
          success *= isEqual(bus.I(n), ScalarT{0.0});
        }

        return success.report(__func__);
      }

      TestOutcome vectorConstruction()
      {
        TestStatus success = true;

        std::vector<RealT>      v0{1.0, -2.0, 3.5, 4.0};
        EMT::Bus<ScalarT, IdxT> bus(7, v0);

        success *= bus.busID() == 7;
        success *= bus.verify() == 0;
        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;
        success *= bus.tagDifferentiable() == 0;
        success *= bus.setAbsoluteTolerance(1.0e-6) == 0;

        success *= bus.phaseCount() == static_cast<IdxT>(v0.size());
        for (std::size_t n = 0; n < v0.size(); ++n)
        {
          success *= isEqual(bus.V(static_cast<IdxT>(n)), static_cast<ScalarT>(v0[n]));
          success *= isEqual(bus.Vp(static_cast<IdxT>(n)), ScalarT{0.0});
          success *= isEqual(bus.tag()[n], ScalarT{1.0});
          success *= isEqual(bus.absoluteTolerance()[n], ScalarT{1.0e-6});
          success *= bus.getVariableIndex(static_cast<IdxT>(n)) == static_cast<IdxT>(n);
          success *= bus.getResidualIndex(static_cast<IdxT>(n)) == static_cast<IdxT>(n);
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(std::vector<RealT>{1.0, 2.0, 3.0});
        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;

        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          bus.I(n) = static_cast<ScalarT>(10 + n);
        }

        success *= bus.evaluateResidual() == 0;
        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          success *= isEqual(bus.I(n), ScalarT{0.0});
        }

        return success.report(__func__);
      }

      TestOutcome currentInjection()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(std::vector<RealT>{1.0, 2.0, 3.0});
        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;

        CurrentInjectionComponent<ScalarT, IdxT> component;
        std::array<ScalarT, 3>                   i_inj{1.0, -2.0, 3.0};

        component.apply(bus, i_inj.data());
        component.apply(bus, i_inj.data());

        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          success *= isEqual(bus.I(n), static_cast<ScalarT>(2.0) * i_inj[static_cast<std::size_t>(n)]);
        }

        return success.report(__func__);
      }

      TestOutcome unsupportedPhasorAliases()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(std::vector<RealT>{1.0, 2.0, 3.0});
        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;

        success *= throws<GridKit::Utilities::NotImplementedError>(
            [&bus]()
            { bus.Vr(); });
        success *= throws<GridKit::Utilities::NotImplementedError>(
            [&bus]()
            { bus.Vi(); });
        success *= throws<GridKit::Utilities::NotImplementedError>(
            [&bus]()
            { bus.Ir(); });
        success *= throws<GridKit::Utilities::NotImplementedError>(
            [&bus]()
            { bus.Ii(); });

        return success.report(__func__);
      }

      TestOutcome systemModel()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        EMT::Bus<ScalarT, IdxT>                    bus(std::vector<RealT>{1.0, 2.0, 3.0});

        system.addBus(&bus);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.evaluateJacobian() == 0;
        success *= system.size() == bus.size();

        return success.report(__func__);
      }

      TestOutcome cooPointerAxpyThreePhase()
      {
        TestStatus success = true;

        GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT> J;

        std::vector<IdxT>  pattern_rows;
        std::vector<IdxT>  pattern_cols;
        std::vector<RealT> pattern_vals;

        pattern_rows.reserve(9);
        pattern_cols.reserve(9);
        pattern_vals.reserve(9);

        for (IdxT r = 0; r < 3; ++r)
        {
          for (IdxT c = 0; c < 3; ++c)
          {
            pattern_rows.push_back(20 + r);
            pattern_cols.push_back(10 + c);
            pattern_vals.push_back(0.0);
          }
        }

        J.setValues(pattern_rows, pattern_cols, pattern_vals);

        std::array<IdxT, 9>  rows{};
        std::array<IdxT, 9>  cols{};
        std::array<RealT, 9> vals{};

        for (IdxT r = 0; r < 3; ++r)
        {
          for (IdxT c = 0; c < 3; ++c)
          {
            const auto k = static_cast<std::size_t>(r * 3 + c);
            rows[k]      = 20 + r;
            cols[k]      = 10 + c;
            vals[k]      = static_cast<RealT>(k + 1);
          }
        }

        J.axpy(2.0, rows.data(), cols.data(), vals.data(), static_cast<IdxT>(vals.size()));

        auto  entries     = J.getEntries();
        auto& entry_rows  = std::get<0>(entries);
        auto& entry_cols  = std::get<1>(entries);
        auto& entry_vals  = std::get<2>(entries);
        auto  expected_nz = vals.size();

        success *= entry_rows.size() == expected_nz;
        success *= entry_cols.size() == expected_nz;
        success *= entry_vals.size() == expected_nz;

        for (std::size_t k = 0; k < entry_vals.size(); ++k)
        {
          success *= entry_rows[k] >= 20;
          success *= entry_rows[k] < 23;
          success *= entry_cols[k] >= 10;
          success *= entry_cols[k] < 13;

          const auto row       = static_cast<std::size_t>(entry_rows[k] - 20);
          const auto col       = static_cast<std::size_t>(entry_cols[k] - 10);
          const auto value_id  = row * 3 + col + 1;
          success             *= isEqual(entry_vals[k], static_cast<RealT>(2 * value_id));
        }

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome jacobian()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(std::vector<RealT>{1.0, 2.0, 3.0});

        success *= bus.allocate() == 0;
        for (IdxT n = 0; n < bus.phaseCount(); ++n)
        {
          bus.setVariableIndex(n, n + 10);
          bus.setResidualIndex(n, n + 20);
        }

        success *= bus.evaluateJacobian() == 0;

        auto  entries = bus.getJacobian().getEntries(false);
        auto& rows    = std::get<0>(entries);
        auto& cols    = std::get<1>(entries);
        auto& vals    = std::get<2>(entries);

        const auto expected_nnz  = static_cast<std::size_t>(bus.phaseCount() * bus.phaseCount());
        success                 *= rows.size() == expected_nnz;
        success                 *= cols.size() == expected_nnz;
        success                 *= vals.size() == expected_nnz;

        for (std::size_t k = 0; k < vals.size(); ++k)
        {
          success *= rows[k] >= 20;
          success *= rows[k] < 20 + bus.phaseCount();
          success *= cols[k] >= 10;
          success *= cols[k] < 10 + bus.phaseCount();
          success *= isEqual(vals[k], RealT{0.0});
        }

        return success.report(__func__);
      }
#endif
    };
  } // namespace Testing
} // namespace GridKit
