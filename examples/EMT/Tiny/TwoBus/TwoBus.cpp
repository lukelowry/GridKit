#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <utility>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

namespace
{
  using Real  = double;
  using Index = size_t;

  using Data   = GridKit::EMT::CaseData<Real, Index>;
  using System = GridKit::EMT::SystemModel<Data>;
  using Ida    = AnalysisManager::Sundials::Ida<Real, Index>;

  namespace Events = GridKit::Model::Events;

  constexpr Real  simulation_end_time = 6.0e-2;
  constexpr int   output_steps        = 600;
  constexpr Real  relative_tolerance  = 1.0e-8;
  constexpr Real  absolute_tolerance  = 1.0e-8;
  constexpr Real  fault_start_time    = 1.0e-2;
  constexpr Real  fault_duration      = 1.0e-3;
  constexpr Real  fault_resistance    = 15.0;
  constexpr Index max_steps           = 200000;

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
    auto loaded        = GridKit::EMT::loadCase<Data>("TwoBus.case.json");
    auto receiving_bus = loaded.names.bus("receiving_bus");

    loaded.data.schedule(faultStartTime(),
                         loaded.data.busRef(receiving_bus),
                         Events::Fault{Events::PhaseMask::abc(), fault_resistance, 0.0, 0.0});
    loaded.data.schedule(faultClearTime(),
                         loaded.data.busRef(receiving_bus),
                         Events::Clear{Events::PhaseMask::abc()});

    return std::move(loaded.data);
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
