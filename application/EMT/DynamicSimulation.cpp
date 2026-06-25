#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

#include "AnalysisUtilities.hpp"

using scalar_type = double;
using index_type  = size_t;

namespace
{
  using SystemT = GridKit::EMT::SystemModel<scalar_type, index_type>;

  void applyFaultEvent(SystemT& system, const GridKit::EMT::BusFaultInterval& fault, bool apply)
  {
    if (apply)
    {
      system.applyBusFault(fault.bus, fault.G);
    }
    else
    {
      system.clearBusFault(fault.bus, fault.G);
    }
  }
} // namespace

int main(int argc, const char* argv[])
{
  try
  {
    GridKit::EMT::checkCommandLine(argc, "DynamicSimulation");

    auto study = GridKit::EMT::parseStudyData(argv[1]);

    GridKit::EMT::SystemModel<scalar_type, index_type> system(study.model_data);
    system.allocate();

    const auto  events      = GridKit::EMT::busFaultEvents(study);
    std::size_t event_index = 0;

    auto applyEventsAt = [&](double time)
    {
      while (event_index < events.size()
             && GridKit::EMT::approximatelyEqual(events[event_index].time, time))
      {
        const auto& event = events[event_index];
        applyFaultEvent(system, study.faults[event.fault_index], event.apply);
        ++event_index;
      }
    };

    applyEventsAt(0.0);

    AnalysisManager::Sundials::Ida<scalar_type, index_type> ida(&system);

    if (study.fixed_step > 0.0)
    {
      ida.setFixedStep(study.fixed_step);
    }
    ida.setMaxSteps(100000);

    ida.configureSimulation();
    ida.initializeSimulation(0.0, study.find_consistent, study.dt);

    GridKit::EMT::TraceWriter trace(study.output_file, study.model_data);
    trace.writeHeader();
    trace.write(0.0, system);

    const auto start = static_cast<double>(clock());

    double curr_time  = 0.0;
    auto   runSegment = [&](double final_time)
    {
      if (GridKit::EMT::approximatelyEqual(final_time, curr_time))
      {
        return;
      }

      const int nout = static_cast<int>(std::round((final_time - curr_time) / study.dt));
      if (nout <= 0)
      {
        throw std::invalid_argument("EMT DynamicSimulation: simulation segment must produce at least one output step");
      }

      ida.runSimulation(final_time,
                        nout,
                        [&](double time)
                        {
                          trace.write(time, system);
                        });
      curr_time = final_time;
    };

    while (event_index < events.size())
    {
      const auto event_time = events[event_index].time;
      runSegment(event_time);
      applyEventsAt(event_time);
      ida.initializeSimulation(event_time, true, study.dt);
    }

    runSegment(study.tmax);

    const auto stop = static_cast<double>(clock());

    std::cout << "\n\nComplete in " << (stop - start) / CLOCKS_PER_SEC << " seconds\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
