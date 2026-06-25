#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

#include "AnalysisUtilities.hpp"

using scalar_type = double;
using index_type  = size_t;

int main(int argc, const char* argv[])
{
  try
  {
    GridKit::EMT::checkCommandLine(argc, "DynamicSimulation");

    auto study = GridKit::EMT::parseStudyData(argv[1]);

    GridKit::EMT::SystemModel<scalar_type, index_type> system(study.model_data);
    system.allocate();

    AnalysisManager::Sundials::Ida<scalar_type, index_type> ida(&system);

    if (study.fixed_step > 0.0)
    {
      ida.setFixedStep(study.fixed_step);
    }

    ida.configureSimulation();
    ida.initializeSimulation(0.0, study.find_consistent);

    GridKit::EMT::TraceWriter trace(study.output_file, study.model_data);
    trace.writeHeader();
    trace.write(0.0, system);

    const auto start = static_cast<double>(clock());
    const int  nout  = static_cast<int>(std::round(study.tmax / study.dt));

    if (nout <= 0)
    {
      throw std::invalid_argument("EMT DynamicSimulation: tmax / dt must produce at least one output step");
    }

    ida.runSimulation(study.tmax,
                      nout,
                      [&](double time)
                      {
                        trace.write(time, system);
                      });

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
