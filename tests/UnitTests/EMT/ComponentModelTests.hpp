#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <GridKit/Model/CallbackVariableMonitor.hpp>
#include <GridKit/Model/EMT/Branch/BranchLumpedConstant/BranchLumpedConstant.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Breaker/Breaker.hpp>
#include <GridKit/Model/EMT/Component/BusFault/BusFault.hpp>
#include <GridKit/Model/EMT/Component/LoadRL/LoadRL.hpp>
#include <GridKit/Model/EMT/Component/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/EMT/System/Network.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/Events.hpp>
#include <GridKit/Model/VariableMonitorController.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class RealT, typename IdxT>
    class EMTComponentModelTests
    {
    public:
      using LoadRL        = EMT::LoadRL<RealT, IdxT>;
      using LoadRLData    = EMT::LoadRLData<RealT, IdxT>;
      using VoltageSource = EMT::VoltageSource<RealT, IdxT>;
      using SourceData    = EMT::VoltageSourceData<RealT, IdxT>;
      using Branch        = EMT::BranchLumpedConstant<RealT, IdxT>;
      using BranchData    = EMT::BranchLumpedConstantData<RealT, IdxT>;
      using Breaker       = EMT::Breaker<RealT, IdxT>;
      using BreakerData   = EMT::BreakerData<RealT, IdxT>;
      using BusFault      = EMT::BusFault<RealT, IdxT>;
      using BusFaultData  = EMT::BusFaultData<RealT, IdxT>;
      using PhaseMask     = GridKit::Model::Events::PhaseMask;

      TestOutcome eventPhaseMask()
      {
        TestStatus success = true;

        success *= (GridKit::Model::Events::Open::name == "open");
        success *= (GridKit::Model::Events::Close::name == "close");
        success *= (GridKit::Model::Events::Fault::name == "fault");
        success *= (GridKit::Model::Events::Clear::name == "clear");

        success *= PhaseMask::a().includes(0);
        success *= !PhaseMask::a().includes(1);
        success *= PhaseMask::b().includes(1);
        success *= PhaseMask::c().includes(2);
        success *= PhaseMask::abc().includes(0);
        success *= PhaseMask::abc().includes(1);
        success *= PhaseMask::abc().includes(2);
        success *= !PhaseMask::abc().includes(3);
        success *= PhaseMask::none().empty();
        success *= (PhaseMask::a().with(PhaseMask::c()).bits() == 0b101);
        success *= (PhaseMask::abc().without(PhaseMask::b()).bits() == 0b101);
        success *= (PhaseMask::fromBits(0b11110000).bits() == 0b000);

        return success.report(__func__);
      }

      TestOutcome busInitialization()
      {
        TestStatus success = true;

        EMT::Bus<RealT, IdxT> bus({120.0, 0.2, 60.0});
        std::array<RealT, 3>  y{};
        std::array<RealT, 3>  yp{};
        bus.initialize(y.data(), yp.data(), IdxT{0});

        const RealT pi    = std::acos(RealT{-1.0});
        const RealT shift = RealT{2.0} * pi / RealT{3.0};
        const RealT w     = RealT{2.0} * pi * RealT{60.0};
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        const RealT tol   = RealT{1.0e-12};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT theta  = RealT{0.2} + (phase == 1 ? -shift : (phase == 2 ? shift : RealT{0.0}));
          success           *= isEqual(y[static_cast<size_t>(phase)], sqrt2 * RealT{120.0} * std::cos(theta), tol);
          success           *= isEqual(yp[static_cast<size_t>(phase)], -sqrt2 * RealT{120.0} * w * std::sin(theta), tol);
        }

        return success.report(__func__);
      }

      TestOutcome loadRLInitialization()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, LoadRL>;
        Network network;

        const IdxT bus  = network.addBus({120.0, 0.25, 60.0});
        const auto load = network.add(LoadRL({{2.0, 3.0, 4.0}, {0.01, 0.02, 0.03}}));
        network.connect(load.terminal(0), bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();
        system.initialize();
        system.evaluateResidual();

        const RealT tol   = RealT{1.0e-10};
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        const auto& slot  = system.layout().component(load);
        const auto& y     = system.y();
        const auto& yp    = system.yp();
        const auto& f     = system.getResidual();

        EMT::Bus<RealT, IdxT> init_bus({120.0, 0.25, 60.0});
        const auto            v = init_bus.template initialVoltagePhasor<RealT>();
        const RealT           w = init_bus.template omega<RealT>();

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const std::complex<RealT> z{loadData().r[static_cast<size_t>(phase)],
                                      w * loadData().l[static_cast<size_t>(phase)]};
          const auto                current = -v[static_cast<size_t>(phase)] / z;
          const RealT               i0      = sqrt2 * current.real();
          const RealT               ip0     = -sqrt2 * w * current.imag();

          success *= isEqual(y[slot.variable_offset + phase], i0, tol);
          success *= isEqual(yp[slot.variable_offset + phase], ip0, tol);
          success *= isEqual(f[slot.equation_offset + phase], RealT{0.0}, tol);
          success *= isEqual(f[system.layout().busEquation(bus, phase)], i0, tol);
        }

        return success.report(__func__);
      }

      TestOutcome voltageSourceResidual()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, VoltageSource>;
        Network network;

        const IdxT bus    = network.addBus({1.0, 0.0, 60.0});
        const auto source = network.add(VoltageSource({{120.0, 121.0, 122.0},
                                                       {0.1, -2.0, 2.2},
                                                       {2.0, 4.0, 5.0},
                                                       2.0 * std::acos(RealT{-1.0}) * 60.0}));
        network.connect(source.terminal(0), bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();
        system.initialize();

        auto& y                                = system.y();
        y[system.layout().busVariable(bus, 0)] = 10.0;
        y[system.layout().busVariable(bus, 1)] = 20.0;
        y[system.layout().busVariable(bus, 2)] = 30.0;

        const RealT t = 0.004;
        system.updateTime(t, 0.0);
        system.evaluateResidual();

        const auto& data  = system.network().components.template get<VoltageSource>()[0].data();
        const auto& f     = system.getResidual();
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        const RealT tol   = RealT{1.0e-12};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT e = sqrt2 * data.e[static_cast<size_t>(phase)]
                          * std::cos(data.omega0 * t + data.phi[static_cast<size_t>(phase)]);
          const RealT ref  = (e - y[system.layout().busVariable(bus, phase)]) / data.r[static_cast<size_t>(phase)];
          success         *= isEqual(f[system.layout().busEquation(bus, phase)], ref, tol);
        }

        return success.report(__func__);
      }

      TestOutcome branchValidation()
      {
        TestStatus success = true;

        success *= throws<std::invalid_argument>(
            [&]()
            {
              auto data   = fullBranchData();
              data.length = RealT{0.0};
              Branch branch(data);
            });

        success *= throws<std::invalid_argument>(
            [&]()
            {
              auto data    = fullBranchData();
              data.r[0][0] = std::numeric_limits<RealT>::infinity();
              Branch branch(data);
            });

        success *= throws<std::invalid_argument>(
            [&]()
            {
              auto data = fullBranchData();
              data.l    = {{{RealT{1.0}, RealT{2.0}, RealT{3.0}},
                            {RealT{2.0}, RealT{4.0}, RealT{6.0}},
                            {RealT{3.0}, RealT{6.0}, RealT{9.0}}}};
              Branch branch(data);
            });

        return success.report(__func__);
      }

      TestOutcome branchInitialization()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Branch>;
        Network network;

        const auto data     = fullBranchData();
        const IdxT from_bus = network.addBus({120.0, 0.2, 60.0});
        const IdxT to_bus   = network.addBus({118.0, 0.05, 60.0});
        const auto branch   = network.add(Branch(data));
        network.connect(branch.terminal(Branch::from), from_bus);
        network.connect(branch.terminal(Branch::to), to_bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();
        system.initialize();
        system.evaluateResidual();

        EMT::Bus<RealT, IdxT> from_init({120.0, 0.2, 60.0});
        EMT::Bus<RealT, IdxT> to_init({118.0, 0.05, 60.0});
        const auto            v_from = from_init.template initialVoltagePhasor<RealT>();
        const auto            v_to   = to_init.template initialVoltagePhasor<RealT>();
        const RealT           omega  = from_init.template omega<RealT>();

        const auto r = EMT::scale(data.r, data.length);
        const auto l = EMT::scale(data.l, data.length);

        EMT::PhaseMatrix<std::complex<RealT>> z{};
        EMT::PhaseVector<std::complex<RealT>> voltage_delta{};
        for (IdxT row = 0; row < 3; ++row)
        {
          voltage_delta[row] = v_from[row] - v_to[row];
          for (IdxT col = 0; col < 3; ++col)
          {
            z[row][col] = {r[row][col], omega * l[row][col]};
          }
        }

        const auto  current = EMT::solve(z, voltage_delta);
        const RealT sqrt2   = std::sqrt(RealT{2.0});
        const auto& slot    = system.layout().component(branch);
        const auto& y       = system.y();
        const auto& yp      = system.yp();
        const auto& f       = system.getResidual();
        const RealT tol     = RealT{1.0e-9};

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT i0  = sqrt2 * current[phase].real();
          const RealT ip0 = -sqrt2 * omega * current[phase].imag();

          success *= isEqual(y[slot.variable_offset + phase], i0, tol);
          success *= isEqual(yp[slot.variable_offset + phase], ip0, tol);
          success *= isEqual(f[slot.equation_offset + phase], RealT{0.0}, tol);
        }

        return success.report(__func__);
      }

      TestOutcome branchResidual()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Branch>;
        Network network;

        const auto data     = fullBranchData();
        const IdxT from_bus = network.addBus({1.0, 0.0, 60.0});
        const IdxT to_bus   = network.addBus({1.0, 0.0, 60.0});
        const auto branch   = network.add(Branch(data));
        network.connect(branch.terminal(Branch::from), from_bus);
        network.connect(branch.terminal(Branch::to), to_bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();

        auto&       y    = system.y();
        auto&       yp   = system.yp();
        const auto& slot = system.layout().component(branch);

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(from_bus, phase)]  = RealT{10.0} + static_cast<RealT>(phase);
          y[system.layout().busVariable(to_bus, phase)]    = RealT{4.0} + RealT{2.0} * static_cast<RealT>(phase);
          yp[system.layout().busVariable(from_bus, phase)] = RealT{1.0} + RealT{0.25} * static_cast<RealT>(phase);
          yp[system.layout().busVariable(to_bus, phase)]   = RealT{2.0} + RealT{0.5} * static_cast<RealT>(phase);
          y[slot.variable_offset + phase]                  = RealT{0.5} + RealT{0.1} * static_cast<RealT>(phase);
          yp[slot.variable_offset + phase]                 = RealT{-0.3} + RealT{0.2} * static_cast<RealT>(phase);
        }

        system.evaluateResidual();

        EMT::PhaseVector<RealT> v_from{};
        EMT::PhaseVector<RealT> v_to{};
        EMT::PhaseVector<RealT> vp_from{};
        EMT::PhaseVector<RealT> vp_to{};
        EMT::PhaseVector<RealT> current{};
        EMT::PhaseVector<RealT> current_derivative{};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          v_from[phase]             = y[system.layout().busVariable(from_bus, phase)];
          v_to[phase]               = y[system.layout().busVariable(to_bus, phase)];
          vp_from[phase]            = yp[system.layout().busVariable(from_bus, phase)];
          vp_to[phase]              = yp[system.layout().busVariable(to_bus, phase)];
          current[phase]            = y[slot.variable_offset + phase];
          current_derivative[phase] = yp[slot.variable_offset + phase];
        }

        const auto r      = EMT::scale(data.r, data.length);
        const auto l      = EMT::scale(data.l, data.length);
        const auto g_half = EMT::scale(data.g, RealT{0.5} * data.length);
        const auto c_half = EMT::scale(data.c, RealT{0.5} * data.length);

        const auto r_current  = EMT::multiply(r, current);
        const auto l_currentp = EMT::multiply(l, current_derivative);
        const auto g_from     = EMT::multiply(g_half, v_from);
        const auto c_from     = EMT::multiply(c_half, vp_from);
        const auto g_to       = EMT::multiply(g_half, v_to);
        const auto c_to       = EMT::multiply(c_half, vp_to);

        const auto& f   = system.getResidual();
        const RealT tol = RealT{1.0e-12};
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT equation       = r_current[phase] + l_currentp[phase] + v_to[phase] - v_from[phase];
          const RealT from_injection = -g_from[phase] - c_from[phase] - current[phase];
          const RealT to_injection   = -g_to[phase] - c_to[phase] + current[phase];

          success *= isEqual(f[slot.equation_offset + phase], equation, tol);
          success *= isEqual(f[system.layout().busEquation(from_bus, phase)], from_injection, tol);
          success *= isEqual(f[system.layout().busEquation(to_bus, phase)], to_injection, tol);
        }

        return success.report(__func__);
      }

      TestOutcome breakerResidual()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Breaker>;
        Network network;

        const IdxT from_bus = network.addBus({1.0, 0.0, 60.0});
        const IdxT to_bus   = network.addBus({1.0, 0.0, 60.0});
        const auto breaker  = network.add(Breaker(BreakerData{}));
        network.connect(breaker.terminal(Breaker::from), from_bus);
        network.connect(breaker.terminal(Breaker::to), to_bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();

        auto&       y    = system.y();
        const auto& slot = system.layout().component(breaker);
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(from_bus, phase)] = RealT{10.0} + static_cast<RealT>(phase);
          y[system.layout().busVariable(to_bus, phase)]   = RealT{4.0} + RealT{2.0} * static_cast<RealT>(phase);
          y[slot.variable_offset + phase]                 = RealT{0.5} + RealT{0.25} * static_cast<RealT>(phase);
        }

        system.evaluateResidual();
        const auto& closed_f = system.getResidual();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const RealT current = y[slot.variable_offset + phase];
          const RealT delta   = y[system.layout().busVariable(to_bus, phase)]
                              - y[system.layout().busVariable(from_bus, phase)];
          success *= isEqual(closed_f[slot.equation_offset + phase], delta, RealT{1.0e-12});
          success *= isEqual(closed_f[system.layout().busEquation(from_bus, phase)], -current, RealT{1.0e-12});
          success *= isEqual(closed_f[system.layout().busEquation(to_bus, phase)], current, RealT{1.0e-12});
        }

        auto& state = system.network().components.template get<Breaker>()[0];
        state.open(PhaseMask::a());
        success *= !state.closed(0);
        success *= state.closed(1);
        success *= state.closed(2);

        system.evaluateResidual();
        const auto& open_f  = system.getResidual();
        success            *= isEqual(open_f[slot.equation_offset + 0],
                           y[slot.variable_offset + 0],
                           RealT{1.0e-12});
        for (IdxT phase = 1; phase < 3; ++phase)
        {
          const RealT delta = y[system.layout().busVariable(to_bus, phase)]
                              - y[system.layout().busVariable(from_bus, phase)];
          success *= isEqual(open_f[slot.equation_offset + phase], delta, RealT{1.0e-12});
        }

        return success.report(__func__);
      }

      TestOutcome breakerEvents()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Breaker>;
        Network network;

        const IdxT from_bus = network.addBus({1.0, 0.0, 60.0});
        const IdxT to_bus   = network.addBus({1.0, 0.0, 60.0});
        const auto breaker  = network.add(Breaker(BreakerData{}));
        network.connect(breaker.terminal(Breaker::from), from_bus);
        network.connect(breaker.terminal(Breaker::to), to_bus);

        network.schedule(0.50, breaker, GridKit::Model::Events::Open{PhaseMask::abc()});
        network.schedule(0.50, breaker, GridKit::Model::Events::Close{PhaseMask::a()});
        network.schedule(0.75, breaker, GridKit::Model::Events::Close{PhaseMask::abc()});

        EMT::SystemModel<Network> system(network);
        system.allocate();

        auto event_time  = system.nextEventTime();
        success         *= event_time.has_value();
        if (event_time)
        {
          success *= isEqual(*event_time, RealT{0.50}, RealT{1.0e-12});
        }

        success     *= system.applyNextEventBatch();
        auto& state  = system.network().components.template get<Breaker>()[0];
        success     *= state.closed(0);
        success     *= !state.closed(1);
        success     *= !state.closed(2);

        event_time  = system.nextEventTime();
        success    *= event_time.has_value();
        if (event_time)
        {
          success *= isEqual(*event_time, RealT{0.75}, RealT{1.0e-12});
        }

        success *= system.applyNextEventBatch();
        success *= state.closed(0);
        success *= state.closed(1);
        success *= state.closed(2);
        success *= !system.nextEventTime().has_value();
        success *= !system.applyNextEventBatch();

        system.resetEventCursor();
        success *= system.nextEventTime().has_value();

        {
          Network                         invalid;
          EMT::TypedComponentRef<Breaker> missing{{0, 0}};
          invalid.schedule(0.10, missing, GridKit::Model::Events::Open{PhaseMask::abc()});
          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> bad_system(invalid);
                bad_system.allocate();
              });
        }

        {
          Network    unsupported;
          const IdxT a      = unsupported.addBus({1.0, 0.0, 60.0});
          const IdxT b      = unsupported.addBus({1.0, 0.0, 60.0});
          const auto target = unsupported.add(Breaker(BreakerData{}));
          unsupported.connect(target.terminal(Breaker::from), a);
          unsupported.connect(target.terminal(Breaker::to), b);
          unsupported.schedule(0.10, target, GridKit::Model::Events::Fault{});
          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> bad_system(unsupported);
                bad_system.allocate();
              });
        }

        {
          Network    negative_time;
          const IdxT a      = negative_time.addBus({1.0, 0.0, 60.0});
          const IdxT b      = negative_time.addBus({1.0, 0.0, 60.0});
          const auto target = negative_time.add(Breaker(BreakerData{}));
          negative_time.connect(target.terminal(Breaker::from), a);
          negative_time.connect(target.terminal(Breaker::to), b);
          negative_time.schedule(-0.10, target, GridKit::Model::Events::Open{PhaseMask::abc()});
          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> bad_system(negative_time);
                bad_system.allocate();
              });
        }

        return success.report(__func__);
      }

      TestOutcome breakerMonitorCsv()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Breaker>;

        const std::string file = "EMTBreakerMonitorTest.csv";
        std::filesystem::remove(file);

        Network    network;
        const IdxT from_bus = network.addBus({1.0, 0.0, 60.0});
        const IdxT to_bus   = network.addBus({1.0, 0.0, 60.0});
        const auto breaker  = network.add(Breaker(BreakerData{}));
        network.connect(breaker.terminal(Breaker::from), from_bus);
        network.connect(breaker.terminal(Breaker::to), to_bus);
        network.addMonitorSink({file, GridKit::Model::VariableMonitorFormat::CSV});
        network.monitorComponent(breaker,
                                 "breaker",
                                 {EMT::BreakerMonitorVariable::ia,
                                  EMT::BreakerMonitorVariable::ib,
                                  EMT::BreakerMonitorVariable::ic,
                                  EMT::BreakerMonitorVariable::dia,
                                  EMT::BreakerMonitorVariable::dib,
                                  EMT::BreakerMonitorVariable::dic});

        EMT::SystemModel<Network> system(network);
        system.allocate();

        const auto& slot = system.layout().component(breaker);
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          system.y()[slot.variable_offset + phase]  = RealT{1.0} + static_cast<RealT>(phase);
          system.yp()[slot.variable_offset + phase] = RealT{4.0} + static_cast<RealT>(phase);
        }
        system.updateTime(0.25, 1.0);
        system.printMonitoredVariables();
        system.stopMonitor();

        std::ifstream input(file);
        std::string   header;
        std::string   row;
        std::getline(input, header);
        std::getline(input, row);
        const auto values = csvNumbers(row);

        success *= (header == "t,breaker_ia,breaker_ib,breaker_ic,breaker_dia,breaker_dib,breaker_dic");
        success *= (values.size() == 7);
        if (values.size() == 7)
        {
          success *= isEqual(values[0], RealT{0.25}, RealT{1.0e-12});
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            success *= isEqual(values[1 + phase],
                               system.y()[slot.variable_offset + phase],
                               RealT{1.0e-12});
            success *= isEqual(values[4 + phase],
                               system.yp()[slot.variable_offset + phase],
                               RealT{1.0e-12});
          }
        }

        std::filesystem::remove(file);
        return success.report(__func__);
      }

      TestOutcome busFaultEvents()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, BusFault>;
        Network network;

        const IdxT bus   = network.addBus({1.0, 0.0, 60.0});
        const auto fault = network.add(BusFault(BusFaultData{}));
        network.connect(fault.terminal(0), bus);
        network.schedule(0.05,
                         fault,
                         GridKit::Model::Events::Fault{PhaseMask::a(), 2.0, 0.0, 0.0});
        network.schedule(0.07, fault, GridKit::Model::Events::Clear{PhaseMask::a()});

        EMT::SystemModel<Network> system(network);
        system.allocate();

        success *= (system.nnz() == 3);

        auto& y = system.y();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(bus, phase)] = RealT{10.0} + RealT{10.0} * static_cast<RealT>(phase);
        }

        system.evaluateResidual();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          success *= isEqual(system.getResidual()[system.layout().busEquation(bus, phase)], RealT{0.0});
        }

        const auto* csr  = system.getCsrJacobian();
        success         *= system.applyNextEventBatch();
        auto& state      = system.network().components.template get<BusFault>()[0];
        success         *= state.active(0);
        success         *= !state.active(1);
        success         *= !state.active(2);

        system.evaluateResidual();
        success *= isEqual(system.getResidual()[system.layout().busEquation(bus, 0)], RealT{-5.0});
        success *= isEqual(system.getResidual()[system.layout().busEquation(bus, 1)], RealT{0.0});
        success *= isEqual(system.getResidual()[system.layout().busEquation(bus, 2)], RealT{0.0});

        system.evaluateJacobian();
        success *= isEqual(csrValue(system,
                                    system.layout().busEquation(bus, 0),
                                    system.layout().busVariable(bus, 0)),
                           RealT{-0.5});
        success *= isEqual(csrValue(system,
                                    system.layout().busEquation(bus, 1),
                                    system.layout().busVariable(bus, 1)),
                           RealT{0.0});

        success *= system.applyNextEventBatch();
        success *= !state.active(0);
        system.evaluateResidual();
        success *= isEqual(system.getResidual()[system.layout().busEquation(bus, 0)], RealT{0.0});
        system.evaluateJacobian();
        success *= (system.getCsrJacobian() == csr);
        success *= (system.nnz() == 3);
        success *= isEqual(csrValue(system,
                                    system.layout().busEquation(bus, 0),
                                    system.layout().busVariable(bus, 0)),
                           RealT{0.0});

        {
          Network    unsupported;
          const IdxT bad_bus   = unsupported.addBus({1.0, 0.0, 60.0});
          const auto bad_fault = unsupported.add(BusFault(BusFaultData{}));
          unsupported.connect(bad_fault.terminal(0), bad_bus);
          unsupported.schedule(0.01, bad_fault, GridKit::Model::Events::Open{PhaseMask::a()});
          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> bad_system(unsupported);
                bad_system.allocate();
              });
        }

        {
          Network    reactive;
          const IdxT bad_bus   = reactive.addBus({1.0, 0.0, 60.0});
          const auto bad_fault = reactive.add(BusFault(BusFaultData{}));
          reactive.connect(bad_fault.terminal(0), bad_bus);
          reactive.schedule(0.01,
                            bad_fault,
                            GridKit::Model::Events::Fault{PhaseMask::a(), 2.0, 1.0, 0.0});
          EMT::SystemModel<Network> reactive_system(reactive);
          reactive_system.allocate();
          success *= throws<std::invalid_argument>(
              [&]()
              {
                reactive_system.applyNextEventBatch();
              });
        }

        return success.report(__func__);
      }

      TestOutcome busFaultMonitorCsv()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, BusFault>;

        const std::string file = "EMTBusFaultMonitorTest.csv";
        std::filesystem::remove(file);

        BusFaultData data{};
        data.active = PhaseMask::abc();
        data.r      = RealT{2.0};

        Network    network;
        const IdxT bus   = network.addBus({1.0, 0.0, 60.0});
        const auto fault = network.add(BusFault(data));
        network.connect(fault.terminal(0), bus);
        network.addMonitorSink({file, GridKit::Model::VariableMonitorFormat::CSV});
        network.monitorComponent(fault,
                                 "fault",
                                 {EMT::BusFaultMonitorVariable::ia,
                                  EMT::BusFaultMonitorVariable::ib,
                                  EMT::BusFaultMonitorVariable::ic});

        EMT::SystemModel<Network> system(network);
        system.allocate();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          system.y()[system.layout().busVariable(bus, phase)] = RealT{10.0} + static_cast<RealT>(phase);
        }
        system.updateTime(0.125, 1.0);
        system.printMonitoredVariables();
        system.stopMonitor();

        std::ifstream input(file);
        std::string   header;
        std::string   row;
        std::getline(input, header);
        std::getline(input, row);
        const auto values = csvNumbers(row);

        success *= (header == "t,fault_ia,fault_ib,fault_ic");
        success *= (values.size() == 4);
        if (values.size() == 4)
        {
          success *= isEqual(values[0], RealT{0.125}, RealT{1.0e-12});
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            const RealT expected  = -system.y()[system.layout().busVariable(bus, phase)] / RealT{2.0};
            success              *= isEqual(values[1 + phase], expected, RealT{1.0e-12});
          }
        }

        std::filesystem::remove(file);
        return success.report(__func__);
      }

      TestOutcome breakerJacobian()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, Breaker>;
        Network network;

        const IdxT from_bus = network.addBus({1.0, 0.0, 60.0});
        const IdxT to_bus   = network.addBus({1.0, 0.0, 60.0});
        const auto breaker  = network.add(Breaker(BreakerData{}));
        network.connect(breaker.terminal(Breaker::from), from_bus);
        network.connect(breaker.terminal(Breaker::to), to_bus);
        network.schedule(0.10, breaker, GridKit::Model::Events::Open{PhaseMask::a()});

        EMT::SystemModel<Network> system(network);
        system.allocate();
        const auto* csr = system.getCsrJacobian();
        const IdxT  nnz = system.nnz();

        success *= (nnz == 15);

        auto&       y    = system.y();
        const auto& slot = system.layout().component(breaker);
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(from_bus, phase)] = RealT{10.0} + static_cast<RealT>(phase);
          y[system.layout().busVariable(to_bus, phase)]   = RealT{4.0} + RealT{2.0} * static_cast<RealT>(phase);
          y[slot.variable_offset + phase]                 = RealT{0.5} + RealT{0.25} * static_cast<RealT>(phase);
        }

        system.evaluateJacobian();
        const IdxT row_a      = slot.equation_offset;
        const IdxT current_a  = slot.variable_offset;
        success              *= isEqual(csrValue(system, row_a, system.layout().busVariable(from_bus, 0)), RealT{-1.0});
        success              *= isEqual(csrValue(system, row_a, system.layout().busVariable(to_bus, 0)), RealT{1.0});
        success              *= isEqual(csrValue(system, row_a, current_a), RealT{0.0});
        success              *= isEqual(csrValue(system, system.layout().busEquation(from_bus, 0), current_a), RealT{-1.0});
        success              *= isEqual(csrValue(system, system.layout().busEquation(to_bus, 0), current_a), RealT{1.0});

        success *= system.applyNextEventBatch();
        system.evaluateJacobian();
        success *= (system.getCsrJacobian() == csr);
        success *= (system.nnz() == nnz);
        success *= isEqual(csrValue(system, row_a, system.layout().busVariable(from_bus, 0)), RealT{0.0});
        success *= isEqual(csrValue(system, row_a, system.layout().busVariable(to_bus, 0)), RealT{0.0});
        success *= isEqual(csrValue(system, row_a, current_a), RealT{1.0});

        const IdxT row_b      = slot.equation_offset + 1;
        const IdxT current_b  = slot.variable_offset + 1;
        success              *= isEqual(csrValue(system, row_b, system.layout().busVariable(from_bus, 1)), RealT{-1.0});
        success              *= isEqual(csrValue(system, row_b, system.layout().busVariable(to_bus, 1)), RealT{1.0});
        success              *= isEqual(csrValue(system, row_b, current_b), RealT{0.0});

        return success.report(__func__);
      }

      TestOutcome discoveredStructure()
      {
        TestStatus success = true;

        {
          using Network = EMT::NetworkData<RealT, IdxT, LoadRL>;
          Network    network;
          const IdxT bus  = network.addBus({120.0, 0.0, 60.0});
          const auto load = network.add(LoadRL(loadData()));
          network.connect(load.terminal(0), bus);

          EMT::SystemModel<Network> system(network);
          system.allocate();
          success *= (system.nnz() == 9);
        }

        {
          using Network = EMT::NetworkData<RealT, IdxT, VoltageSource>;
          Network    network;
          const IdxT bus    = network.addBus({120.0, 0.0, 60.0});
          const auto source = network.add(VoltageSource(sourceData()));
          network.connect(source.terminal(0), bus);

          EMT::SystemModel<Network> system(network);
          system.allocate();
          success *= (system.nnz() == 3);
        }

        {
          using Network = EMT::NetworkData<RealT, IdxT, Branch>;
          Network    network;
          const IdxT from_bus = network.addBus({120.0, 0.0, 60.0});
          const IdxT to_bus   = network.addBus({118.0, 0.1, 60.0});
          const auto branch   = network.add(Branch(fullBranchData()));
          network.connect(branch.terminal(Branch::from), from_bus);
          network.connect(branch.terminal(Branch::to), to_bus);

          EMT::SystemModel<Network> system(network);
          system.allocate();
          success *= (system.nnz() == 39);
        }

        {
          using Network = EMT::NetworkData<RealT, IdxT, Branch>;
          Network    network;
          const IdxT from_bus = network.addBus({120.0, 0.0, 60.0});
          const IdxT to_bus   = network.addBus({118.0, 0.1, 60.0});
          const auto branch   = network.add(Branch(diagonalBranchData()));
          network.connect(branch.terminal(Branch::from), from_bus);
          network.connect(branch.terminal(Branch::to), to_bus);

          EMT::SystemModel<Network> system(network);
          system.allocate();
          success *= (system.nnz() == 21);
        }

        return success.report(__func__);
      }

      TestOutcome zeroStateSparsity()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, LoadRL, VoltageSource, Branch>;
        Network network;

        const IdxT from_bus = network.addBus({120.0, 0.15, 60.0});
        const IdxT to_bus   = network.addBus({118.0, 0.02, 60.0});
        const auto load     = network.add(LoadRL(loadData()));
        const auto source   = network.add(VoltageSource(sourceData()));
        const auto branch   = network.add(Branch(fullBranchData()));
        network.connect(load.terminal(0), to_bus);
        network.connect(source.terminal(0), from_bus);
        network.connect(branch.terminal(Branch::from), from_bus);
        network.connect(branch.terminal(Branch::to), to_bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();

        std::fill(system.y().begin(), system.y().end(), RealT{0.0});
        std::fill(system.yp().begin(), system.yp().end(), RealT{0.0});
        system.updateTime(0.0, RealT{5.0});

        try
        {
          system.evaluateJacobian();
        }
        catch (...)
        {
          success *= false;
        }

        success *= (system.nnz() > 0);
        return success.report(__func__);
      }

      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, LoadRL, VoltageSource, Branch>;
        Network network;

        const IdxT from_bus = network.addBus({120.0, 0.15, 60.0});
        const IdxT to_bus   = network.addBus({118.0, 0.02, 60.0});
        const auto load     = network.add(LoadRL(loadData()));
        const auto source   = network.add(VoltageSource(sourceData()));
        const auto branch   = network.add(Branch(fullBranchData()));
        network.connect(load.terminal(0), to_bus);
        network.connect(source.terminal(0), from_bus);
        network.connect(branch.terminal(Branch::from), from_bus);
        network.connect(branch.terminal(Branch::to), to_bus);

        EMT::SystemModel<Network> system(network);
        system.allocate();
        system.initialize();
        const RealT alpha = RealT{7.0};
        system.updateTime(0.003, alpha);

        auto& y  = system.y();
        auto& yp = system.yp();
        for (IdxT i = 0; i < system.size(); ++i)
        {
          y[static_cast<size_t>(i)]  += RealT{0.01} * static_cast<RealT>(i + 1);
          yp[static_cast<size_t>(i)] += RealT{0.02} * static_cast<RealT>(i + 1);
        }

        success *= checkJacobian(system, alpha, RealT{1.0e-7}, RealT{1.0e-5});

        return success.report(__func__);
      }

      TestOutcome callbackVariableMonitor()
      {
        TestStatus success = true;

        const std::string file = "EMTCallbackMonitorTest.csv";
        std::filesystem::remove(file);

        RealT time = RealT{0.5};
        RealT a    = RealT{1.25};
        RealT b    = RealT{-2.5};

        GridKit::Model::VariableMonitorController<RealT> controller(time);
        GridKit::Model::CallbackVariableMonitor<RealT>   monitor("sample");
        monitor.add("a", [&]()
                    { return a; });
        monitor.add("b", [&]()
                    { return b; });

        controller.addSink({file, GridKit::Model::VariableMonitorFormat::CSV});
        controller.addMonitor(&monitor);
        controller.start();
        controller.print();
        controller.stop();

        std::ifstream input(file);
        std::string   header;
        std::string   row;
        std::getline(input, header);
        std::getline(input, row);
        const auto values = csvNumbers(row);

        success *= (header == "t,sample_a,sample_b");
        success *= (values.size() == 3);
        if (values.size() == 3)
        {
          success *= isEqual(values[0], time, RealT{1.0e-12});
          success *= isEqual(values[1], a, RealT{1.0e-12});
          success *= isEqual(values[2], b, RealT{1.0e-12});
        }

        std::filesystem::remove(file);
        return success.report(__func__);
      }

      TestOutcome emtMonitorCsv()
      {
        TestStatus success = true;

        using Network = EMT::NetworkData<RealT, IdxT, LoadRL, VoltageSource, Branch>;

        const std::string file = "EMTMonitorBindingTest.csv";
        std::filesystem::remove(file);

        Network    network;
        const IdxT from_bus = network.addBus({120.0, 0.15, 60.0});
        const IdxT to_bus   = network.addBus({118.0, 0.02, 60.0});
        const auto load     = network.add(LoadRL(loadData()));
        const auto source   = network.add(VoltageSource(sourceData()));
        const auto branch   = network.add(Branch(fullBranchData()));
        network.connect(load.terminal(0), to_bus);
        network.connect(source.terminal(0), from_bus);
        network.connect(branch.terminal(Branch::from), from_bus);
        network.connect(branch.terminal(Branch::to), to_bus);

        network.addMonitorSink({file, GridKit::Model::VariableMonitorFormat::CSV});
        network.monitorBus(from_bus, "source_bus", {EMT::BusMonitorVariable::va, EMT::BusMonitorVariable::vb, EMT::BusMonitorVariable::vc});
        network.monitorBus(to_bus, "load_bus", {EMT::BusMonitorVariable::va, EMT::BusMonitorVariable::vb, EMT::BusMonitorVariable::vc});
        network.monitorComponent(source, "source", {EMT::VoltageSourceMonitorVariable::ia, EMT::VoltageSourceMonitorVariable::ib, EMT::VoltageSourceMonitorVariable::ic});
        network.monitorComponent(load, "load", {EMT::LoadRLMonitorVariable::ia, EMT::LoadRLMonitorVariable::ib, EMT::LoadRLMonitorVariable::ic});
        network.monitorComponent(branch, "line", {EMT::BranchLumpedConstantMonitorVariable::ia, EMT::BranchLumpedConstantMonitorVariable::ib, EMT::BranchLumpedConstantMonitorVariable::ic});

        EMT::SystemModel<Network> system(network);
        system.allocate();
        system.initialize();
        system.updateTime(0.0, 1.0);
        system.evaluateResidual();
        system.printMonitoredVariables();
        system.stopMonitor();

        std::ifstream input(file);
        std::string   header;
        std::string   row;
        std::getline(input, header);
        std::getline(input, row);
        const auto values = csvNumbers(row);

        success *= (header == "t,source_bus_va,source_bus_vb,source_bus_vc,load_bus_va,load_bus_vb,load_bus_vc,source_ia,source_ib,source_ic,load_ia,load_ib,load_ic,line_ia,line_ib,line_ic");
        success *= (values.size() == 16);

        if (values.size() == 16)
        {
          const auto& y          = system.y();
          const auto& sourceData = system.network().components.template get<VoltageSource>()[0].data();
          const auto& loadSlot   = system.layout().component(load);
          const auto& lineSlot   = system.layout().component(branch);
          const RealT sqrt2      = std::sqrt(RealT{2.0});

          success *= isEqual(values[0], RealT{0.0}, RealT{1.0e-12});
          for (IdxT phase = 0; phase < 3; ++phase)
          {
            success *= isEqual(values[1 + phase],
                               y[system.layout().busVariable(from_bus, phase)],
                               RealT{1.0e-12});
            success *= isEqual(values[4 + phase],
                               y[system.layout().busVariable(to_bus, phase)],
                               RealT{1.0e-12});

            const RealT e = sqrt2 * sourceData.e[phase] * std::cos(sourceData.phi[phase]);
            const RealT sourceCurrent =
                (e - y[system.layout().busVariable(from_bus, phase)]) / sourceData.r[phase];
            success *= isEqual(values[7 + phase], sourceCurrent, RealT{1.0e-12});
            success *= isEqual(values[10 + phase],
                               y[loadSlot.variable_offset + phase],
                               RealT{1.0e-12});
            success *= isEqual(values[13 + phase],
                               y[lineSlot.variable_offset + phase],
                               RealT{1.0e-12});
          }
        }

        std::filesystem::remove(file);
        return success.report(__func__);
      }

      TestOutcome emtMonitorValidation()
      {
        TestStatus success = true;

        {
          using Network = EMT::NetworkData<RealT, IdxT, LoadRL>;
          Network network;
          network.addBus({120.0, 0.0, 60.0});
          network.addMonitorSink({"unused.csv", GridKit::Model::VariableMonitorFormat::CSV});
          network.monitorBus(IdxT{99}, "missing", {EMT::BusMonitorVariable::va});

          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> system(network);
                system.allocate();
              });
        }

        {
          using Network = EMT::NetworkData<RealT, IdxT, LoadRL>;
          Network    network;
          const IdxT bus  = network.addBus({120.0, 0.0, 60.0});
          const auto load = network.add(LoadRL(loadData()));
          network.connect(load.terminal(0), bus);
          network.addMonitorSink({"unused.csv", GridKit::Model::VariableMonitorFormat::CSV});
          network.monitorComponent(load, "load", {static_cast<EMT::LoadRLMonitorVariable>(99)});

          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> system(network);
                system.allocate();
              });
        }

        {
          using Network = EMT::NetworkData<RealT, IdxT, LoadRL>;
          Network network;
          network.addMonitorSink({"unused.csv", GridKit::Model::VariableMonitorFormat::CSV});
          EMT::TypedComponentRef<LoadRL> missing{{99, 0}};
          network.monitorComponent(missing, "missing", {EMT::LoadRLMonitorVariable::ia});

          success *= throws<std::invalid_argument>(
              [&]()
              {
                EMT::SystemModel<Network> system(network);
                system.allocate();
              });
        }

        std::filesystem::remove("unused.csv");
        return success.report(__func__);
      }

    private:
      template <class System>
      RealT csrValue(System& system, IdxT row, IdxT col) const
      {
        auto*        csr      = system.getCsrJacobian();
        const IdxT*  row_ptrs = csr->getRowData();
        const IdxT*  cols     = csr->getColData();
        const RealT* values   = csr->getValues();
        for (IdxT k = row_ptrs[row]; k < row_ptrs[row + 1]; ++k)
        {
          if (cols[k] == col)
          {
            return values[k];
          }
        }
        return RealT{0.0};
      }

      std::vector<RealT> csvNumbers(const std::string& row) const
      {
        std::vector<RealT> values;
        std::stringstream  stream(row);
        std::string        field;
        while (std::getline(stream, field, ','))
        {
          values.push_back(static_cast<RealT>(std::stod(field)));
        }
        return values;
      }

      template <class System>
      bool checkJacobian(System& system, RealT alpha, RealT eps, RealT tol)
      {
        TestStatus success = true;

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

        auto& y  = system.y();
        auto& yp = system.yp();

        std::vector<RealT> base_y  = y;
        std::vector<RealT> base_yp = yp;
        system.evaluateResidual();
        const std::vector<RealT> base_f = system.getResidual();

        for (IdxT col = 0; col < system.size(); ++col)
        {
          y                             = base_y;
          yp                            = base_yp;
          y[static_cast<size_t>(col)]  += eps;
          yp[static_cast<size_t>(col)] += alpha * eps;
          system.evaluateResidual();

          for (IdxT row = 0; row < system.size(); ++row)
          {
            const RealT fd   = (system.getResidual()[static_cast<size_t>(row)] - base_f[static_cast<size_t>(row)]) / eps;
            const RealT val  = valueAt(row, col);
            success         *= isEqual(val, fd, tol);
          }
        }

        y  = base_y;
        yp = base_yp;
        return success;
      }

      LoadRLData loadData() const
      {
        return {{2.0, 3.0, 4.0}, {0.01, 0.02, 0.03}};
      }

      SourceData sourceData() const
      {
        const RealT pi = std::acos(RealT{-1.0});
        return {{120.0, 121.0, 122.0}, {0.1, 0.1 - 2.0 * pi / 3.0, 0.1 + 2.0 * pi / 3.0}, {5.0, 6.0, 7.0}, 2.0 * pi * 60.0};
      }

      BranchData fullBranchData() const
      {
        BranchData data{};
        data.length = RealT{10.0};
        data.r      = {{{RealT{0.20}, RealT{0.02}, RealT{0.01}},
                        {RealT{0.02}, RealT{0.21}, RealT{0.03}},
                        {RealT{0.01}, RealT{0.03}, RealT{0.22}}}};
        data.l      = {{{RealT{0.010}, RealT{0.001}, RealT{0.002}},
                        {RealT{0.001}, RealT{0.011}, RealT{0.0015}},
                        {RealT{0.002}, RealT{0.0015}, RealT{0.012}}}};
        data.g      = {{{RealT{0.0010}, RealT{0.0001}, RealT{0.0002}},
                        {RealT{0.0001}, RealT{0.0011}, RealT{0.00015}},
                        {RealT{0.0002}, RealT{0.00015}, RealT{0.0012}}}};
        data.c      = {{{RealT{1.0e-6}, RealT{1.0e-7}, RealT{2.0e-7}},
                        {RealT{1.0e-7}, RealT{1.1e-6}, RealT{1.5e-7}},
                        {RealT{2.0e-7}, RealT{1.5e-7}, RealT{1.2e-6}}}};
        return data;
      }

      BranchData diagonalBranchData() const
      {
        BranchData data{};
        data.length = RealT{10.0};
        data.r      = {{{RealT{0.20}, RealT{0.0}, RealT{0.0}},
                        {RealT{0.0}, RealT{0.21}, RealT{0.0}},
                        {RealT{0.0}, RealT{0.0}, RealT{0.22}}}};
        data.l      = {{{RealT{0.010}, RealT{0.0}, RealT{0.0}},
                        {RealT{0.0}, RealT{0.011}, RealT{0.0}},
                        {RealT{0.0}, RealT{0.0}, RealT{0.012}}}};
        data.c      = {{{RealT{1.0e-6}, RealT{0.0}, RealT{0.0}},
                        {RealT{0.0}, RealT{1.1e-6}, RealT{0.0}},
                        {RealT{0.0}, RealT{0.0}, RealT{1.2e-6}}}};
        return data;
      }
    };
  } // namespace Testing
} // namespace GridKit
