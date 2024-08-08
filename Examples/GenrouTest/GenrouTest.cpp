// Made by Paul Moon 6/7/2024


#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>

#include <ComponentLib/PowerFlow/Bus/BusSlack.hpp>
#include <ComponentLib/DynamicPhasor/SynchronousMachine/GENROUwS/GENROU.hpp>
#include <SystemModel.hpp>
#include <Solver/Dynamic/Ida.hpp>
#include <Solver/Optimization/DynamicObjective.hpp>
#include <Solver/Optimization/DynamicConstraint.hpp>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <Utilities/Testing.hpp>

int main()
{
    using namespace ModelLib;
    using namespace AnalysisManager::Sundials;
    using namespace AnalysisManager;
    using namespace GridKit::Testing;
    
    // Create a slack bus
    BaseBus<double, size_t>* bus = new BusSlack<double, size_t>(1.0, 0.0); 

    // Attach a generator to that bus
    GENROU<double, size_t>* gen = new GENROU<double, size_t>(bus);

    // Create a system model
    SystemModel<double, size_t>* model = new SystemModel<double, size_t>();
    model->addBus(bus);
    model->addComponent(gen);

    // allocate model components
    model->allocate();

    std::cout << "Size: " << model->y().size() << std::endl;

    model->initialize();
    model->evaluateResidual();

    std::cout << "Verify Initial Residual is Zero: {";
    for (double i : model->getResidual())
    {
        std::cout << i << ", ";
    }
    std::cout << "}" << std::endl;

    model->updateTime(0.0, 1.0);
    model->evaluateJacobian();

    // Create numerical integrator and configure it for the generator model
    AnalysisManager::Sundials::Ida<double, size_t>* idas = new AnalysisManager::Sundials::Ida<double, size_t>(model);

    double t_init  = 0.0;
    double t_final = 5.0;

    idas->configureSimulation();
    idas->getDefaultInitialCondition();
    idas->initializeSimulation(t_init, true);
    idas->runSimulation(t_final, 1000);

    delete idas;
    delete model;
    return 0;
}