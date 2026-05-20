#pragma once

#include <filesystem>
#include <variant>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    class EMTJsonParserTests
    {
    public:
      TestOutcome parseTwoBusCase()
      {
        TestStatus success = true;

        auto input_file = std::filesystem::current_path() / "TwoBus.case.json";
        auto data       = EMT::parseSystemModelData(input_file);

        success *= (data.bus.size() == 2);
        success *= (data.branch_lumped_constant.size() == 1);
        success *= (data.load_rl.size() == 1);
        success *= (data.voltage_source.size() == 1);
        success *= (data.breaker.size() == 1);
        success *= (data.monitor_sink.size() == 1);

        success *= (data.bus[0].name == "source_bus");
        success *= (data.bus[1].name == "receiving_bus");
        success *= (data.branch_lumped_constant[0].ports.at(EMT::BranchLumpedConstantPorts::from) == 0);
        success *= (data.branch_lumped_constant[0].ports.at(EMT::BranchLumpedConstantPorts::to) == 1);
        success *= (data.load_rl[0].ports.at(EMT::LoadRLPorts::ac) == 1);
        success *= (data.voltage_source[0].ports.at(EMT::VoltageSourcePorts::bus) == 0);
        success *= (data.breaker[0].ports.at(EMT::BreakerPorts::from) == 0);
        success *= (data.breaker[0].ports.at(EMT::BreakerPorts::to) == 1);

        auto r = std::get<EMT::PhaseVector<double>>(
            data.load_rl[0].parameters.at(EMT::LoadRLParameters::r));
        success *= isEqual(r[0], 24.9);

        EMT::SystemModel<double, size_t> system(data);
        success *= (system.allocate() == 0);
        success *= (system.size() == 15);
        success *= (system.initialize() == 0);
        success *= (system.tagDifferentiable() == 0);
        success *= (system.evaluateResidual() == 0);
        success *= (system.evaluateJacobian() == 0);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
