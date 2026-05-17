#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Component/Branch/BranchFrequencyDependent/BranchFrequencyDependent.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class RealT, typename IdxT>
    class EMTFrequencyDependentBranchTests
    {
    public:
      using Branch     = EMT::BranchFrequencyDependent<RealT, IdxT>;
      using BranchData = EMT::BranchFrequencyDependentData<RealT, IdxT>;
      using Data       = EMT::SystemModelData<RealT, IdxT, Branch>;

      TestOutcome constantDResidual()
      {
        TestStatus success = true;

        Data data;
        const IdxT from_bus = data.addBus({1.0, 0.0});
        const IdxT to_bus   = data.addBus({1.0, 0.0});
        const auto branch   = data.add(Branch(branchData({2.0, 0.0, 0.0,
                                                          0.0, 3.0, 0.0,
                                                          0.0, 0.0, 4.0})));
        data.connect(branch.port(Branch::from), from_bus);
        data.connect(branch.port(Branch::to), to_bus);

        EMT::SystemModel<Data> system(data);
        system.allocate();

        auto& y  = system.y();
        auto& yp = system.yp();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(from_bus, phase)] = RealT{1.0} + static_cast<RealT>(phase);
          y[system.layout().busVariable(to_bus, phase)]   = RealT{4.0} + static_cast<RealT>(phase);
          yp[system.layout().busVariable(from_bus, phase)] = RealT{0.0};
          yp[system.layout().busVariable(to_bus, phase)]   = RealT{0.0};
        }

        system.evaluateResidual();
        const auto& f   = system.getResidual();
        const RealT tol = RealT{1.0e-12};

        success *= isEqual(f[system.layout().busEquation(from_bus, 0)], RealT{-2.0}, tol);
        success *= isEqual(f[system.layout().busEquation(from_bus, 1)], RealT{-6.0}, tol);
        success *= isEqual(f[system.layout().busEquation(from_bus, 2)], RealT{-12.0}, tol);
        success *= isEqual(f[system.layout().busEquation(to_bus, 0)], RealT{-8.0}, tol);
        success *= isEqual(f[system.layout().busEquation(to_bus, 1)], RealT{-15.0}, tol);
        success *= isEqual(f[system.layout().busEquation(to_bus, 2)], RealT{-24.0}, tol);

        return success.report(__func__);
      }

      TestOutcome zeroFitResidual()
      {
        TestStatus success = true;

        Data data;
        const IdxT from_bus = data.addBus({1.0, 0.0});
        const IdxT to_bus   = data.addBus({1.0, 0.0});
        const auto branch   = data.add(Branch(branchData({0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.0})));
        data.connect(branch.port(Branch::from), from_bus);
        data.connect(branch.port(Branch::to), to_bus);

        EMT::SystemModel<Data> system(data);
        system.allocate();
        std::fill(system.y().begin(), system.y().end(), RealT{5.0});
        std::fill(system.yp().begin(), system.yp().end(), RealT{0.0});
        system.evaluateResidual();

        success *= (maxAbs(system.getResidual()) < RealT{1.0e-12});
        return success.report(__func__);
      }

      TestOutcome monitorCurrents()
      {
        TestStatus success = true;

        const std::string file = "EMTFrequencyDependentBranchMonitor.csv";
        std::filesystem::remove(file);

        Data data;
        const IdxT from_bus = data.addBus({1.0, 0.0});
        const IdxT to_bus   = data.addBus({1.0, 0.0});
        const auto branch   = data.add(Branch(branchData({1.0, 0.0, 0.0,
                                                          0.0, 1.0, 0.0,
                                                          0.0, 0.0, 1.0})));
        data.connect(branch.port(Branch::from), from_bus);
        data.connect(branch.port(Branch::to), to_bus);
        data.addMonitorSink({file, GridKit::Model::VariableMonitorFormat::CSV});
        data.monitorComponent(branch,
                              "line",
                              {EMT::BranchFrequencyDependentMonitorVariable::ifa,
                               EMT::BranchFrequencyDependentMonitorVariable::ifb,
                               EMT::BranchFrequencyDependentMonitorVariable::ifc,
                               EMT::BranchFrequencyDependentMonitorVariable::ita,
                               EMT::BranchFrequencyDependentMonitorVariable::itb,
                               EMT::BranchFrequencyDependentMonitorVariable::itc});

        EMT::SystemModel<Data> system(data);
        system.allocate();
        auto& y = system.y();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          y[system.layout().busVariable(from_bus, phase)] = RealT{1.0} + static_cast<RealT>(phase);
          y[system.layout().busVariable(to_bus, phase)]   = RealT{4.0} + static_cast<RealT>(phase);
        }
        system.updateTime(0.0, 1.0);
        system.printMonitoredVariables();
        system.stopMonitor();

        std::ifstream input(file);
        std::string   header;
        std::string   row;
        std::getline(input, header);
        std::getline(input, row);
        const auto values = csvNumbers(row);

        success *= (header == "t,line_ifa,line_ifb,line_ifc,line_ita,line_itb,line_itc");
        success *= (values.size() == 7u);
        if (values.size() == 7u)
        {
          success *= isEqual(values[1], RealT{-1.0}, RealT{1.0e-12});
          success *= isEqual(values[2], RealT{-2.0}, RealT{1.0e-12});
          success *= isEqual(values[3], RealT{-3.0}, RealT{1.0e-12});
          success *= isEqual(values[4], RealT{-4.0}, RealT{1.0e-12});
          success *= isEqual(values[5], RealT{-5.0}, RealT{1.0e-12});
          success *= isEqual(values[6], RealT{-6.0}, RealT{1.0e-12});
        }

        input.close();
        std::filesystem::remove(file);
        return success.report(__func__);
      }

    private:
      static BranchData branchData(const std::array<RealT, 9>& d)
      {
        BranchData data;
        data.length      = RealT{1000.0};
        data.phase_order = {"a", "b", "c"};
        data.yc.dimension = 3u;
        data.yc.d.assign(d.begin(), d.end());
        data.yc.e.assign(9u, RealT{0.0});
        return data;
      }

      static RealT maxAbs(const std::vector<RealT>& values)
      {
        RealT norm = 0.0;
        for (const auto& value : values)
        {
          norm = std::max(norm, std::abs(value));
        }
        return norm;
      }

      static std::vector<RealT> csvNumbers(const std::string& row)
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
    };
  } // namespace Testing
} // namespace GridKit
