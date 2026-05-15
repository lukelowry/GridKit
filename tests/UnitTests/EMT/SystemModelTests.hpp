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
      using OneElectricalPort    = EMTMocks::MockOneElectricalPortComponent<RealT, IdxT>;
      using BranchLumpedConstant = EMTMocks::MockBranchLumpedConstant<RealT, IdxT>;
      using ThreeElectricalPort  = EMTMocks::MockThreeElectricalPortBranch<RealT, IdxT>;
      using Source               = EMTMocks::MockPortSource<RealT, IdxT>;
      using Sink                 = EMTMocks::MockPortSink<RealT, IdxT>;
      using MissingDifferential  = EMTMocks::MockMissingDifferential<RealT, IdxT>;
      using ZeroDerivative       = EMTMocks::MockZeroDerivativeNonlinear<RealT, IdxT>;
      using Dynamic              = EMTMocks::MockDynamicComponent<RealT, IdxT>;

      TestOutcome systemModelData()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneElectricalPort, Source>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0});
        const IdxT b         = data.addBus({1.0, 0.1});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneElectricalPort{});
        data.connect(branch.port(BranchLumpedConstant::from), a);
        data.connect(branch.port(BranchLumpedConstant::to), b);
        data.connect(component.port(0), a);

        success *= (a == 0);
        success *= (b == 1);
        success *= (branch.id.type == 0 && branch.id.index == 0);
        success *= (component.id.type == 1 && component.id.index == 0);
        success *= (data.buses.size() == 2);
        success *= (data.components.template get<BranchLumpedConstant>().size() == 1);
        success *= (data.components.template get<OneElectricalPort>().size() == 1);
        success *= (data.port_connections.size() == 3);

        EMT::SystemModelData<RealT, IdxT> bus_only;
        bus_only.addBus({1.0, 0.0});
        EMT::SystemModel<decltype(bus_only)> bus_system(bus_only);
        bus_system.allocate();
        success *= (bus_system.size() == 3);
        success *= (!EMT::ComponentTraits<MissingDifferential>::is_valid);

        return success.report(__func__);
      }

      TestOutcome layout()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneElectricalPort>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0});
        const IdxT b         = data.addBus({1.0, 0.0});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneElectricalPort{});
        data.connect(branch.port(BranchLumpedConstant::from), a);
        data.connect(branch.port(BranchLumpedConstant::to), b);
        data.connect(component.port(0), b);

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
        success *= (system.layout().component(branch).electrical_port_offset == 0);
        success *= (system.layout().component(branch).electrical_port_count == 2);
        success *= (system.layout().component(component).electrical_port_offset == 2);
        success *= (system.layout().component(component).electrical_port_count == 1);
        success *= (!system.tag()[0]);
        success *= (system.tag()[6]);
        success *= (system.tag()[9]);

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, BranchLumpedConstant, OneElectricalPort>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0});
        const IdxT b         = data.addBus({1.0, 0.0});
        const auto branch    = data.add(BranchLumpedConstant{});
        const auto component = data.add(OneElectricalPort{});
        data.connect(branch.port(BranchLumpedConstant::from), a);
        data.connect(branch.port(BranchLumpedConstant::to), b);
        data.connect(component.port(0), a);

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

      TestOutcome threeElectricalPortComponent()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, ThreeElectricalPort>;
        Data data;

        const IdxT a         = data.addBus({1.0, 0.0});
        const IdxT b         = data.addBus({1.0, 0.0});
        const IdxT c         = data.addBus({1.0, 0.0});
        const auto component = data.add(ThreeElectricalPort{});
        data.connect(component.port(0), a);
        data.connect(component.port(1), b);
        data.connect(component.port(2), c);

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

      TestOutcome portWiring()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, OneElectricalPort>;

        Data missing;
        missing.addBus({1.0, 0.0});
        missing.add(OneElectricalPort{});
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> system(missing);
              system.allocate();
            });

        Data       duplicate;
        const IdxT duplicate_bus       = duplicate.addBus({1.0, 0.0});
        const auto duplicate_component = duplicate.add(OneElectricalPort{});
        duplicate.connect(duplicate_component.port(0), duplicate_bus);
        duplicate.connect(duplicate_component.port(0), duplicate_bus);
        success *= throws<std::invalid_argument>(
            [&]()
            {
              EMT::SystemModel<Data> system(duplicate);
              system.allocate();
            });

        Data       invalid_bus;
        const auto invalid_component = invalid_bus.add(OneElectricalPort{});
        invalid_bus.connect(invalid_component.port(0), IdxT{99});
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
        data.connect(source.outputPort(0), sink.inputPort(0));

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
        duplicate_input.connect(duplicate_source.outputPort(0), duplicate_sink.inputPort(0));
        duplicate_input.connect(duplicate_source.outputPort(0), duplicate_sink.inputPort(0));
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

        using Data = EMT::SystemModelData<RealT, IdxT, OneElectricalPort>;
        Data data;

        const IdxT bus       = data.addBus({1.0, 0.0});
        const auto component = data.add(OneElectricalPort{});
        data.connect(component.port(0), bus);

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

      TestOutcome zeroDerivativeGeneralPattern()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, ZeroDerivative>;
        Data       data;
        const auto component = data.add(ZeroDerivative{});

        EMT::SystemModel<Data> system(data);
        system.allocate();

        success *= (system.nnz() == 1);

        system.y()[system.layout().component(component).variable_offset] = RealT{0.20};
        system.updateTime(0.0, RealT{0.0});
        system.evaluateJacobian();

        auto*        csr       = system.getCsrJacobian();
        const RealT* values    = csr->getValues();
        const RealT  expected  = RealT{2.0} * (RealT{0.20} - RealT{0.15625});
        success               *= isEqual(values[0], expected, RealT{1.0e-12});

        return success.report(__func__);
      }

      TestOutcome dynamicComponent()
      {
        TestStatus success = true;

        using Data = EMT::SystemModelData<RealT, IdxT, OneElectricalPort, Dynamic>;
        Data data;

        const IdxT bus               = data.addBus({1.0, 0.0});
        const auto static_component  = data.add(OneElectricalPort{});
        const auto dynamic_component = data.add(Dynamic{4});
        data.connect(static_component.port(0), bus);

        EMT::SystemModel<Data> system(data);
        system.allocate();
        system.initialize();
        system.tagDifferentiable();

        const auto& dynamic_slot  = system.layout().component(dynamic_component);
        success                  *= (dynamic_slot.variable_count == 4);
        success                  *= (dynamic_slot.equation_count == 4);
        success                  *= (system.size() == 10);

        for (IdxT i = 0; i < dynamic_slot.variable_count; ++i)
        {
          success *= system.tag()[static_cast<size_t>(dynamic_slot.variable_offset + i)];
          success *= isEqual(system.y()[static_cast<size_t>(dynamic_slot.variable_offset + i)],
                             static_cast<RealT>(i + 1));
          success *= isEqual(system.yp()[static_cast<size_t>(dynamic_slot.variable_offset + i)],
                             static_cast<RealT>(100 + i));
        }

        auto& y  = system.y();
        auto& yp = system.yp();
        for (IdxT i = 0; i < dynamic_slot.variable_count; ++i)
        {
          y[static_cast<size_t>(dynamic_slot.variable_offset + i)]  = static_cast<RealT>(i + 2);
          yp[static_cast<size_t>(dynamic_slot.variable_offset + i)] = static_cast<RealT>(10 + i);
        }

        system.evaluateResidual();
        const auto& f = system.getResidual();
        for (IdxT i = 0; i < dynamic_slot.equation_count; ++i)
        {
          const RealT expected  = static_cast<RealT>(10 + i) + RealT{2.0} * static_cast<RealT>(i + 2);
          success              *= isEqual(f[static_cast<size_t>(dynamic_slot.equation_offset + i)], expected);
        }

        system.updateTime(0.0, RealT{3.0});
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

        success *= isEqual(valueAt(dynamic_slot.equation_offset, dynamic_slot.variable_offset),
                           RealT{5.0});

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
