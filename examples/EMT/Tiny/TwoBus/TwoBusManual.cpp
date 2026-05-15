#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <utility>

#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
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
  using Data       = GridKit::EMT::SystemModelData<Real, Index, LoadRL, Source, Branch>;
  using System     = GridKit::EMT::SystemModel<Data>;
  using Ida        = AnalysisManager::Sundials::Ida<Real, Index>;
  using Complex    = std::complex<Real>;
  using Format     = GridKit::Model::VariableMonitorFormat;
  using BusVar     = GridKit::EMT::BusMonitorVariable;
  using LoadVar    = GridKit::EMT::LoadRLMonitorVariable;
  using SourceVar  = GridKit::EMT::VoltageSourceMonitorVariable;
  using LineVar    = GridKit::EMT::BranchLumpedConstantMonitorVariable;

  namespace Events = GridKit::Model::Events;

  constexpr Real        frequency           = 60.0;
  constexpr Real        simulation_end_time = 6.0e-2;
  constexpr int         output_steps        = 600;
  constexpr Real        relative_tolerance  = 1.0e-8;
  constexpr Real        absolute_tolerance  = 1.0e-8;
  constexpr Real        fault_start_time    = 1.0e-2;
  constexpr Real        fault_duration      = 1.0e-3;
  constexpr Real        fault_resistance    = 15.0;
  constexpr Real        branch_shunt_g_half = 1.0e-4;
  constexpr Real        branch_shunt_c_half = 5.0e-5;
  constexpr const char* output_file         = "EMTTinyTwoBusManual.csv";
  constexpr Index       max_steps           = 200000;

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
    return fault_start_time;
  }

  Real faultClearTime()
  {
    return faultStartTime() + fault_duration;
  }

  Data makeData()
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

    Data        data;
    const Index source_bus    = data.addBus({std::abs(source_bus_voltage),
                                             std::arg(source_bus_voltage)});
    const Index receiving_bus = data.addBus({std::abs(load_voltage),
                                             std::arg(load_voltage)});

    const auto source = data.add(Source(sourceData(source_internal_voltage)));
    const auto load   = data.add(LoadRL(loadData()));
    const auto branch = data.add(Branch(branchData()));

    data.connect(source.port(0), source_bus);
    data.connect(load.port(0), receiving_bus);
    data.connect(branch.port(Branch::from), source_bus);
    data.connect(branch.port(Branch::to), receiving_bus);

    data.schedule(faultStartTime(),
                  data.busRef(receiving_bus),
                  Events::Fault{Events::PhaseMask::abc(), fault_resistance, 0.0, 0.0});
    data.schedule(faultClearTime(),
                  data.busRef(receiving_bus),
                  Events::Clear{Events::PhaseMask::abc()});

    data.addMonitorSink({output_file, Format::CSV});
    data.monitorBus(source_bus, "source_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    data.monitorBus(receiving_bus, "receiving_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    data.monitorComponent(source, "source", {SourceVar::ia, SourceVar::ib, SourceVar::ic});
    data.monitorComponent(load, "load", {LoadVar::ia, LoadVar::ib, LoadVar::ic});
    data.monitorComponent(branch, "line", {LineVar::ia, LineVar::ib, LineVar::ic});
    data.monitorBus(receiving_bus, "fault", {BusVar::ifa, BusVar::ifb, BusVar::ifc});

    return data;
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
  auto data = makeData();

  System system(std::move(data), relative_tolerance, absolute_tolerance, true, max_steps);
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
    ida.initializeSimulation(*event_time, false);
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

  std::cout << "Example: EMT Tiny TwoBus Manual\n";
  std::cout << "state size      : " << system.size() << '\n';
  std::cout << "jacobian nnz    : " << system.nnz() << '\n';
  std::cout << "output file     : " << output_file << '\n';
  std::cout << "fault start     : " << faultStartTime() << '\n';
  std::cout << "fault clear     : " << faultClearTime() << '\n';
  std::cout << "output rows     : " << output_steps + 1 + event_output_rows << '\n';
  std::cout << "initial residual: " << initial_residual_norm << '\n';
  std::cout << "final residual  : " << residual_norm << '\n';

  return simulation_status == 0 && initial_residual_norm < 1.0e-8 ? EXIT_SUCCESS : EXIT_FAILURE;
}
