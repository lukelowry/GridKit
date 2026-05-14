#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <utility>

#include <GridKit/Model/EMT/Branch/BranchLumpedConstant/BranchLumpedConstant.hpp>
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
  using Network    = GridKit::EMT::NetworkData<Real, Index, LoadRL, Source, Branch>;
  using System     = GridKit::EMT::SystemModel<Network>;
  using Ida        = AnalysisManager::Sundials::Ida<Real, Index>;
  using Complex    = std::complex<Real>;
  using Format     = GridKit::Model::VariableMonitorFormat;
  using BusVar     = GridKit::EMT::BusMonitorVariable;
  using LoadVar    = GridKit::EMT::LoadRLMonitorVariable;
  using SourceVar  = GridKit::EMT::VoltageSourceMonitorVariable;
  using LineVar    = GridKit::EMT::BranchLumpedConstantMonitorVariable;

  constexpr Real frequency           = 60.0;
  constexpr Real simulation_end_time = 0.1;
  constexpr int  output_steps        = 600;
  constexpr Real relative_tolerance  = 1.0e-8;
  constexpr Real absolute_tolerance  = 1.0e-8;

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

  Network makeNetwork()
  {
    const Real    omega = 2.0 * pi() * frequency;
    const Complex load_voltage{120.0, 0.0};
    const Complex load_impedance{25.0, omega * 5.0e-2};
    const Complex branch_impedance{0.10, omega * 1.0e-3};
    const Real    source_resistance = 0.10;

    const Complex load_current            = load_voltage / load_impedance;
    const Complex source_bus_voltage      = load_voltage + branch_impedance * load_current;
    const Complex source_internal_voltage = source_bus_voltage + source_resistance * load_current;

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

    network.connect(source.terminal(0), source_bus);
    network.connect(load.terminal(0), load_bus);
    network.connect(branch.terminal(Branch::from), source_bus);
    network.connect(branch.terminal(Branch::to), load_bus);

    network.addMonitorSink({"EMTTinyTwoBus.csv", Format::CSV});
    network.monitorBus(source_bus, "source_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    network.monitorBus(load_bus, "load_bus", {BusVar::va, BusVar::vb, BusVar::vc});
    network.monitorComponent(source, "source", {SourceVar::ia, SourceVar::ib, SourceVar::ic});
    network.monitorComponent(load, "load", {LoadVar::ia, LoadVar::ib, LoadVar::ic});
    network.monitorComponent(branch, "line", {LineVar::ia, LineVar::ib, LineVar::ic});

    return network;
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

  system.printMonitoredVariables();
  const int simulation_status = ida.runSimulation(simulation_end_time, output_steps);

  system.evaluateResidual();
  system.stopMonitor();

  const Real residual_norm = maxAbs(system.getResidual());

  std::cout << "Example: EMT Tiny TwoBus\n";
  std::cout << "state size      : " << system.size() << '\n';
  std::cout << "jacobian nnz    : " << system.nnz() << '\n';
  std::cout << "output file     : EMTTinyTwoBus.csv\n";
  std::cout << "output rows     : " << output_steps + 1 << '\n';
  std::cout << "initial residual: " << initial_residual_norm << '\n';
  std::cout << "final residual  : " << residual_norm << '\n';

  return simulation_status == 0 && initial_residual_norm < 1.0e-8 ? EXIT_SUCCESS : EXIT_FAILURE;
}
