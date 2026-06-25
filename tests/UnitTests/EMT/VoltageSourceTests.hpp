#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceDataJSONParser.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/Utilities/MapFromCOO.hpp>
#endif

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTVoltageSourceTests
    {
    public:
      using SourceT = EMT::VoltageSource<ScalarT, IdxT, 3>;
      using RealT   = typename SourceT::RealT;
      using BusT    = typename SourceT::BusT;
      using DataT   = typename SourceT::ModelDataT;

      TestOutcome parser()
      {
        TestStatus success = true;

        nlohmann::json j = {
            {"class", "VoltageSource"},
            {"id", "src1"},
            {"params",
             {{"E", {120.0, 121.0, 122.0}},
              {"phi", {0.0, -2.0, 2.0}},
              {"omega", 377.0},
              {"G", {1000.0, 1001.0, 1002.0}}}}};

        auto data = j.get<DataT>();

        success *= data.device_class == "VoltageSource";
        success *= data.disambiguation_string == "src1";
        success *= isEqual(data.E[1], RealT{121.0});
        success *= isEqual(data.phi[2], RealT{2.0});
        success *= isEqual(data.omega, RealT{377.0});
        success *= isEqual(data.G[0], RealT{1000.0});

        return success.report(__func__);
      }

      TestOutcome parserRejectsInvalidParameters()
      {
        TestStatus success = true;

        nlohmann::json base = {
            {"class", "VoltageSource"},
            {"id", "bad"},
            {"params",
             {{"E", {120.0, 120.0, 120.0}},
              {"phi", {0.0, -2.0, 2.0}},
              {"omega", 377.0},
              {"G", {1.0, 1.0, 1.0}}}}};

        auto bad_e                   = base;
        bad_e["params"]["E"][1]      = -1.0;
        auto bad_omega               = base;
        bad_omega["params"]["omega"] = 0.0;
        auto bad_g                   = base;
        bad_g["params"]["G"][2]      = 0.0;

        success *= throws<std::invalid_argument>(
            [&bad_e]()
            { bad_e.get<DataT>(); });
        success *= throws<std::invalid_argument>(
            [&bad_omega]()
            { bad_omega.get<DataT>(); });
        success *= throws<std::invalid_argument>(
            [&bad_g]()
            { bad_g.get<DataT>(); });

        return success.report(__func__);
      }

      TestOutcome verifyRejectsInvalidConfiguration()
      {
        TestStatus success = true;

        BusT  bus2(2, std::vector<RealT>{1.0, 2.0});
        BusT  bus3(3, std::vector<RealT>{1.0, 2.0, 3.0});
        DataT data = makeData();

        SourceT missing_bus(nullptr, data);
        success *= missing_bus.verify() > 0;

        SourceT mismatched_bus(&bus2, data);
        success *= mismatched_bus.verify() > 0;

        data.G[0] = 0.0;
        SourceT bad_g(&bus3, data);
        success *= bad_g.verify() > 0;

        return success.report(__func__);
      }

      TestOutcome residualAndTime()
      {
        TestStatus success = true;

        BusT    bus(1, std::vector<RealT>{10.0, 20.0, -30.0});
        DataT   data = makeData();
        SourceT source(&bus, data);

        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;
        success *= source.allocate() == 0;
        success *= source.initialize() == 0;

        success *= bus.evaluateResidual() == 0;
        success *= source.evaluateResidual() == 0;

        for (std::size_t n = 0; n < 3; ++n)
        {
          success *= isEqual(bus.I(static_cast<IdxT>(n)),
                             expectedCurrent(data, bus.V(static_cast<IdxT>(n)), RealT{0.0}, n));
        }

        source.updateTime(RealT{0.5}, RealT{0.0});
        success *= bus.evaluateResidual() == 0;
        success *= source.evaluateResidual() == 0;

        for (std::size_t n = 0; n < 3; ++n)
        {
          success *= isEqual(bus.I(static_cast<IdxT>(n)),
                             expectedCurrent(data, bus.V(static_cast<IdxT>(n)), RealT{0.5}, n));
        }

        return success.report(__func__);
      }

      TestOutcome systemModel()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        BusT                                       bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        SourceT                                    source(&bus, makeData());

        system.addBus(&bus);
        system.addComponent(&source);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.size() == bus.size() + source.size();

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        BusT    bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        DataT   data = makeData();
        SourceT source(&bus, data);

        success *= bus.allocate() == 0;
        success *= source.allocate() == 0;

        for (IdxT n = 0; n < bus.size(); ++n)
        {
          bus.setVariableIndex(n, static_cast<IdxT>(100 + n));
          bus.setResidualIndex(n, static_cast<IdxT>(200 + n));
        }

        success *= bus.evaluateJacobian() == 0;
        source.updateTime(0.0, 4.0);
        success *= source.evaluateJacobian() == 0;

        bus.getJacobian().deduplicate();
        auto actual = GridKit::Testing::MapFromCOO(bus.getJacobian());

        for (std::size_t n = 0; n < 3; ++n)
        {
          const auto row  = static_cast<std::size_t>(200 + n);
          const auto col  = static_cast<IdxT>(100 + n);
          success        *= isEqual(actual[row][col], -data.G[n]);
        }

        return success.report(__func__);
      }
#endif

    private:
      static DataT makeData()
      {
        DataT data;
        data.E     = {100.0, 110.0, 120.0};
        data.phi   = {0.0, 0.1, -0.2};
        data.omega = 2.0;
        data.G     = {0.1, 0.2, 0.3};
        return data;
      }

      static ScalarT expectedCurrent(const DataT& data, ScalarT voltage, RealT time, std::size_t n)
      {
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        const RealT v_src = sqrt2 * data.E[n] * std::cos(data.omega * time + data.phi[n]);
        return data.G[n] * (v_src - voltage);
      }
    };
  } // namespace Testing
} // namespace GridKit
