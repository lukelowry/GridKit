#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <utility>

#include <GridKit/Model/EMT/Branch/BranchLumpedConstant/BranchLumpedConstant.hpp>
#include <GridKit/Model/EMT/Component/BusFault/BusFault.hpp>
#include <GridKit/Model/EMT/Component/LoadRL/LoadRL.hpp>
#include <GridKit/Model/EMT/Component/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/EMT/System/Network.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

namespace
{
  using Real  = double;
  using Index = size_t;

  using Branch     = GridKit::EMT::BranchLumpedConstant<Real, Index>;
  using BranchData = GridKit::EMT::BranchLumpedConstantData<Real, Index>;
  using LoadRL     = GridKit::EMT::LoadRL<Real, Index>;
  using LoadRLData = GridKit::EMT::LoadRLData<Real, Index>;
  using Source     = GridKit::EMT::VoltageSource<Real, Index>;
  using SourceData = GridKit::EMT::VoltageSourceData<Real, Index>;
  using BusFault   = GridKit::EMT::BusFault<Real, Index>;
  using FaultData  = GridKit::EMT::BusFaultData<Real, Index>;
  using Network    = GridKit::EMT::NetworkData<Real, Index, LoadRL, Source, Branch, BusFault>;
  using System     = GridKit::EMT::SystemModel<Network>;
  using Ida        = AnalysisManager::Sundials::Ida<Real, Index>;
  using Complex    = std::complex<Real>;
  using Format     = GridKit::Model::VariableMonitorFormat;
  using BusVar     = GridKit::EMT::BusMonitorVariable;
  using LoadVar    = GridKit::EMT::LoadRLMonitorVariable;
  using SourceVar  = GridKit::EMT::VoltageSourceMonitorVariable;
  using LineVar    = GridKit::EMT::BranchLumpedConstantMonitorVariable;
  using FaultVar   = GridKit::EMT::BusFaultMonitorVariable;

  namespace Events = GridKit::Model::Events;

  constexpr Real frequency           = 60.0;
  constexpr Real simulation_end_time = 0.1;
  constexpr int  output_steps        = 600;
  constexpr Real relative_tolerance  = 1.0e-8;
  constexpr Real absolute_tolerance  = 1.0e-8;
  constexpr Real fault_start_cycles  = 3.0;
  constexpr Real fault_duration      = 1.0e-3;
  constexpr Real fault_resistance    = 5.0;
  constexpr Real branch_shunt_g_half = 1.0e-4;
  constexpr Real branch_shunt_c_half = 5.0e-5;

  Real pi()
  {
    return std::acos(Real{-1.0});
  }

  BranchData branchData()
  {
    BranchData data{};
    data.length = 1.0;
    for (Index phase = 0; phase < 3; ++phase)
    {
      data.r[phase][phase] = 0.10;
      data.l[phase][phase] = 1.0e-3;
      data.g[phase][phase] = 2.0 * branch_shunt_g_half / data.length;
      data.c[phase][phase] = 2.0 * branch_shunt_c_half / data.length;
    }
    return data;
  }

  LoadRLData loadData()
  {
    return {{25.0, 25.0, 25.0}, {5.0e-2, 5.0e-2, 5.0e-2}};
  }

  SourceData sourceData(Complex source_internal_voltage)
  {
    const Real magnitude = std::abs(source_internal_voltage);
    const Real angle     = std::arg(source_internal_voltage);
    const Real shift     = 2.0 * pi() / 3.0;

    return {{magnitude, magnitude, magnitude},
            {angle, angle - shift, angle + shift},
            {0.10, 0.10, 0.10},
            2.0 * pi() * frequency};
  }

  Real faultStartTime()
  {
    return fault_start_cycles / frequency;
  }

  Real faultClearTime()
  {
    return faultStartTime() + fault_duration;
  }

  Network makeNetwork()
  {
    const Real    omega = 2.0 * pi() * frequency;
    const Complex load_voltage{120.0, 0.0};
    const Complex load_impedance{25.0, omega * 5.0e-2};
    const Complex branch_impedance{0.10, omega * 1.0e-3};
    const Complex branch_shunt_admittance{branch_shunt_g_half,
                                          omega * branch_shunt_c_half};
    const Real    source_resistance = 0.10;

    const Complex load_injection          = -load_voltage / load_impedance;
    const Complex branch_current          = branch_shunt_admittance * load_voltage - load_injection;
    const Complex source_bus_voltage      = load_voltage + branch_impedance * branch_current;
    const Complex source_injection        = branch_shunt_admittance * source_bus_voltage + branch_current;
    const Complex source_internal_voltage = source_bus_voltage + source_resistance * source_injection;

    Network     network;
    const Index source_bus = network.addBus({std::abs(source_bus_voltage),
                                             std::arg(source_bus_voltage),
                                             frequency});
    const Index load_bus   = network.addBus({std::abs(load_voltage),
                                             std::arg(load_voltage),
                                             frequency});

    const auto source = network.add(Source(sourceData(source_internal_voltage)));
    const auto load   = network.add(LoadRL(loadData()));
    const auto branch = network.add(Branch(branchData()));
    const auto fault  = network.add(BusFault(FaultData{}));

    network.connect(source.terminal(0), source_bus);
    network.connect(load.terminal(0), load_bus);
    network.connect(branch.terminal(Branch::from), source_bus);
    network.connect(branch.terminal(Branch::to), load_bus);
    network.connect(fault.terminal(0), load_bus);

    network.schedule(faultStartTime(),
                     fault,
                     Events::Fault{Events::PhaseMask::abc(), fault_resistance, 0.0, 0.0});
    network.schedule(faultClearTime(),
                     fault,
                     Events::Clear{Events::PhaseMask::abc()});

    network.addMonitorSink({"EMTTinyTwoBus.csv", Format::CSV});
    network.monitorBus(source_bus, "source_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    network.monitorBus(load_bus, "load_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    network.monitorComponent(source, "source", {SourceVar::ia, SourceVar::ib, SourceVar::ic});
    network.monitorComponent(load, "load", {LoadVar::ia, LoadVar::ib, LoadVar::ic});
    network.monitorComponent(branch, "line", {LineVar::ia, LineVar::ib, LineVar::ic});
    network.monitorComponent(fault, "fault", {FaultVar::ia, FaultVar::ib, FaultVar::ic});

    return network;
  }

  Real consistencyTout(Real event_time)
  {
    constexpr Real event_consistency_window = 1.0e-5;
    return event_time + event_consistency_window;
  }

  int outputSteps(Real start_time, Real end_time)
  {
    if (end_time <= start_time)
    {
      return 0;
    }

    const Real step = simulation_end_time / static_cast<Real>(output_steps);
    return std::max(1, static_cast<int>(std::lround((end_time - start_time) / step)));
  }

  template <class Vector>
  Real maxAbs(const Vector& values)
  {
    Real norm = 0.0;
    for (const auto& value : values)
    {
      norm = std::max(norm, std::abs(value));
    }
    return norm;
  }
} // namespace

int main()
{
  auto network = makeNetwork();

  System system(std::move(network), relative_tolerance, absolute_tolerance);
  system.allocate();

  Ida ida(&system);
  ida.configureSimulation();
  ida.initializeSimulation(0.0, false);

  system.updateTime(0.0, 1.0);
  system.evaluateResidual();
  system.evaluateJacobian();

  const Real initial_residual_norm = maxAbs(system.getResidual());

  int  simulation_status = 0;
  int  event_output_rows = 0;
  Real current_time      = 0.0;

  system.printMonitoredVariables();
  while (simulation_status == 0)
  {
    const auto event_time = system.nextEventTime();
    if (!event_time || *event_time > simulation_end_time)
    {
      break;
    }

    const int segment_steps = outputSteps(current_time, *event_time);
    if (segment_steps > 0)
    {
      simulation_status = ida.runSimulation(*event_time, segment_steps);
    }
    if (simulation_status != 0)
    {
      break;
    }

    system.applyNextEventBatch();
    const bool clear_event = std::abs(*event_time - faultClearTime()) < 1.0e-12;
    ida.initializeSimulation(*event_time, clear_event, consistencyTout(*event_time));
    system.updateTime(*event_time, 0.0);
    system.evaluateResidual();
    system.printMonitoredVariables();
    ++event_output_rows;
    current_time = *event_time;
  }

  if (simulation_status == 0)
  {
    const int segment_steps = outputSteps(current_time, simulation_end_time);
    if (segment_steps > 0)
    {
      simulation_status = ida.runSimulation(simulation_end_time, segment_steps);
    }
  }

  system.evaluateResidual();
  system.stopMonitor();

  const Real residual_norm = maxAbs(system.getResidual());

  std::cout << "Example: EMT Tiny TwoBus\n";
  std::cout << "state size      : " << system.size() << '\n';
  std::cout << "jacobian nnz    : " << system.nnz() << '\n';
  std::cout << "output file     : EMTTinyTwoBus.csv\n";
  std::cout << "fault start     : " << faultStartTime() << '\n';
  std::cout << "fault clear     : " << faultClearTime() << '\n';
  std::cout << "output rows     : " << output_steps + 1 + event_output_rows << '\n';
  std::cout << "initial residual: " << initial_residual_norm << '\n';
  std::cout << "final residual  : " << residual_norm << '\n';

  return simulation_status == 0 && initial_residual_norm < 1.0e-8 ? EXIT_SUCCESS : EXIT_FAILURE;
}
