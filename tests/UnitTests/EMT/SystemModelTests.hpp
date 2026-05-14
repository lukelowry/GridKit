#pragma once

#include <stdexcept>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "MockModels.hpp"

namespace GridKit
{
  namespace Testing
  {
    template <class RealT, typename IdxT>
    class EMTSystemModelTests
    {
    public:
      using OneTerminal          = EMTMocks::MockOneTerminalComponent<RealT, IdxT>;
      using BranchLumpedConstant = EMTMocks::MockBranchLumpedConstant<RealT, IdxT>;
      using ThreeTerminal        = EMTMocks::MockThreeTerminalBranch<RealT, IdxT>;
      using Source               = EMTMocks::MockPortSource<RealT, IdxT>;
      using Sink                 = EMTMocks::MockPortSink<RealT, IdxT>;
      using MissingDifferential  = EMTMocks::MockMissingDifferential<RealT, IdxT>;

      TestOutcome systemModelData()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneTerminal, Source>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0, 60.0});
        const IdxT b         = data.addBus({1.0, 0.1, 60.0});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneTerminal{});
        data.connect(branch.terminal(BranchLumpedConstant::from), a);
        data.connect(branch.terminal(BranchLumpedConstant::to), b);
        data.connect(component.terminal(0), a);

        success *= (a == 0);
        success *= (b == 1);
        success *= (branch.id.type == 0 && branch.id.index == 0);
        success *= (component.id.type == 1 && component.id.index == 0);
        success *= (data.buses.size() == 2);
        success *= (data.components.template get<BranchLumpedConstant>().size() == 1);
        success *= (data.components.template get<OneTerminal>().size() == 1);
        success *= (data.terminal_connections.size() == 3);

        EMT::SystemModelData<RealT, IdxT> bus_only;
        bus_only.addBus({1.0, 0.0, 60.0});
        EMT::SystemModel<decltype(bus_only)> bus_system(bus_only);
        bus_system.allocate();
        success *= (bus_system.size() == 3);
        success *= (!EMT::ComponentTraits<MissingDifferential>::is_valid);

        return success.report(__func__);
      }

      TestOutcome layout()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneTerminal>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0, 60.0});
        const IdxT b         = data.addBus({1.0, 0.0, 60.0});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneTerminal{});
        data.connect(branch.terminal(BranchLumpedConstant::from), a);
        data.connect(branch.terminal(BranchLumpedConstant::to), b);
        data.connect(component.terminal(0), b);

        EMT::SystemModel<Data> system(data);
        system.allocate();
        system.tagDifferentiable();

        success *= (system.size() == 12);
        success *= (system.layout().busVariable(a, 0) == 0);
        success *= (system.layout().busVariable(b, 0) == 3);
        success *= (system.layout().component(branch).variable_offset == 6);
        success *= (system.layout().component(component).variable_offset == 9);
        success *= (system.layout().component(branch).equation_offset == 6);
        success *= (system.layout().component(component).equation_offset == 9);
        success *= (system.layout().component(branch).terminal_offset == 0);
        success *= (system.layout().component(branch).terminal_count == 2);
        success *= (system.layout().component(component).terminal_offset == 2);
        success *= (system.layout().component(component).terminal_count == 1);
        success *= (!system.tag()[0]);
        success *= (system.tag()[6]);
        success *= (system.tag()[9]);

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneTerminal>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0, 60.0});
        const IdxT b         = data.addBus({1.0, 0.0, 60.0});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneTerminal{});
        data.connect(branch.terminal(BranchLumpedConstant::from), a);
        data.connect(branch.terminal(BranchLumpedConstant::to), b);
        data.connect(component.terminal(0), a);

        EMT::SystemModel<Data> system(data);
        system.allocate();

        auto& y                                                     = system.y();
        y[system.layout().busVariable(a, 0)]                        = 10.0;
        y[system.layout().busVariable(a, 1)]                        = 20.0;
        y[system.layout().busVariable(a, 2)]                        = 30.0;
        y[system.layout().busVariable(b, 0)]                        = 1.0;
        y[system.layout().busVariable(b, 1)]                        = 2.0;
        y[system.layout().busVariable(b, 2)]                        = 3.0;
        y[system.layout().component(branch).variable_offset + 0]    = 4.0;
        y[system.layout().component(branch).variable_offset + 1]    = 5.0;
        y[system.layout().component(branch).variable_offset + 2]    = 6.0;
        y[system.layout().component(component).variable_offset + 0] = 7.0;
        y[system.layout().component(component).variable_offset + 1] = 8.0;
        y[system.layout().component(component).variable_offset + 2] = 9.0;

        system.evaluateResidual();
        const auto& f = system.getResidual();

        success *= isEqual(f[0], RealT{3.0});
        success *= isEqual(f[1], RealT{3.0});
        success *= isEqual(f[2], RealT{3.0});
        success *= isEqual(f[3], RealT{4.0});
        success *= isEqual(f[4], RealT{5.0});
        success *= isEqual(f[5], RealT{6.0});
        success *= isEqual(f[system.layout().component(branch).equation_offset + 0], RealT{-5.0});
        success *= isEqual(f[system.layout().component(branch).equation_offset + 1], RealT{-13.0});
        success *= isEqual(f[system.layout().component(branch).equation_offset + 2], RealT{-21.0});
        success *= isEqual(f[system.layout().component(component).equation_offset + 0], RealT{17.0});
        success *= isEqual(f[system.layout().component(component).equation_offset + 1], RealT{28.0});
        success *= isEqual(f[system.layout().component(component).equation_offset + 2], RealT{39.0});

        return success.report(__func__);
      }

      TestOutcome threeTerminalComponent()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, ThreeTerminal>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0, 60.0});
        const IdxT b         = data.addBus({1.0, 0.0, 60.0});
        const IdxT c         = data.addBus({1.0, 0.0, 60.0});
        const auto component = data.add(ThreeTerminal{});
        data.connect(component.terminal(0), a);
        data.connect(component.terminal(1), b);
        data.connect(component.terminal(2), c);

        EMT::SystemModel<Data> system(data);
        system.allocate();
        system.evaluateResidual();

        const auto& f = system.getResidual();
        for (IdxT i = 0; i < 9; ++i)
        {
          success *= isEqual(f[i], static_cast<RealT>(i + 1));
        }

        return success.report(__func__);
      }

      TestOutcome terminalWiring()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, OneTerminal>;

        Data missing;
        missing.addBus({1.0, 0.0, 60.0});
        missing.add(OneTerminal{});
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> system(missing);
              system.allocate();
            });

        Data       duplicate;
        const IdxT duplicate_bus       = duplicate.addBus({1.0, 0.0, 60.0});
        const auto duplicate_component = duplicate.add(OneTerminal{});
        duplicate.connect(duplicate_component.terminal(0), duplicate_bus);
        duplicate.connect(duplicate_component.terminal(0), duplicate_bus);
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> system(duplicate);
              system.allocate();
            });

        Data       invalid_bus;
        const auto invalid_component = invalid_bus.add(OneTerminal{});
        invalid_bus.connect(invalid_component.terminal(0), IdxT{99});
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> system(invalid_bus);
              system.allocate();
            });

        return success.report(__func__);
      }

      TestOutcome ports()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, Source, Sink>;
        Data data;

        const auto source = data.add(Source{});
        const auto sink   = data.add(Sink{});
        data.connect(source.output(0), sink.input(0));

        EMT::SystemModel<Data> system(data);
        system.allocate();
        system.y()[system.layout().component(source).variable_offset] = 11.0;
        system.y()[system.layout().component(sink).variable_offset]   = 13.0;
        system.evaluateResidual();

        success *= isEqual(system.getResidual()[system.layout().component(source).equation_offset], RealT{1.0});
        success *= isEqual(system.getResidual()[system.layout().component(sink).equation_offset], RealT{2.0});
        success *= (system.nnz() == 3);

        system.evaluateJacobian();
        auto*        csr      = system.getCsrJacobian();
        const IdxT*  row_ptrs = csr->getRowData();
        const IdxT*  cols     = csr->getColData();
        const RealT* values   = csr->getValues();

        const auto valueAt = [&](IdxT row, IdxT col)
        {
          for (IdxT k = row_ptrs[row]; k < row_ptrs[row + 1]; ++k)
          {
            if (cols[k] == col)
            {
              return values[k];
            }
          }
          return RealT{0.0};
        };

        const IdxT source_variable  = system.layout().component(source).variable_offset;
        const IdxT sink_variable    = system.layout().component(sink).variable_offset;
        const IdxT sink_equation    = system.layout().component(sink).equation_offset;
        success                    *= isEqual(valueAt(sink_equation, sink_variable), RealT{1.0});
        success                    *= isEqual(valueAt(sink_equation, source_variable), RealT{-1.0});

        Data missing_input;
        missing_input.add(Source{});
        missing_input.add(Sink{});
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> bad_system(missing_input);
              bad_system.allocate();
            });

        Data       duplicate_input;
        const auto duplicate_source = duplicate_input.add(Source{});
        const auto duplicate_sink   = duplicate_input.add(Sink{});
        duplicate_input.connect(duplicate_source.output(0), duplicate_sink.input(0));
        duplicate_input.connect(duplicate_source.output(0), duplicate_sink.input(0));
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> bad_system(duplicate_input);
              bad_system.allocate();
            });

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, OneTerminal>;
        Data data;

        const IdxT bus       = data.addBus({1.0, 0.0, 60.0});
        const auto component = data.add(OneTerminal{});
        data.connect(component.terminal(0), bus);

        EMT::SystemModel<Data> system(data);
        system.allocate();
        auto*      csr = system.getCsrJacobian();
        const IdxT nnz = system.nnz();

        system.updateTime(0.0, 5.0);
        system.evaluateJacobian();

        success *= (system.getCsrJacobian() == csr);
        success *= (system.nnz() == nnz);

        const IdxT*  row_ptrs = csr->getRowData();
        const IdxT*  cols     = csr->getColData();
        const RealT* values   = csr->getValues();

        const auto valueAt = [&](IdxT row, IdxT col)
        {
          for (IdxT k = row_ptrs[row]; k < row_ptrs[row + 1]; ++k)
          {
            if (cols[k] == col)
            {
              return values[k];
            }
          }
          return RealT{0.0};
        };

        success *= (system.nnz() == 9);

        const IdxT s0  = system.layout().component(component).variable_offset;
        const IdxT r0  = system.layout().component(component).equation_offset;
        success       *= isEqual(valueAt(r0, s0), RealT{1.0});
        success       *= isEqual(valueAt(r0, system.layout().busVariable(bus, 0)), RealT{1.0});
        success       *= isEqual(valueAt(system.layout().busEquation(bus, 0), s0), RealT{1.0});

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
