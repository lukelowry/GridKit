#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRL.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRLDataJSONParser.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSource.hpp>
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
    class EMTLoadRLTests
    {
    public:
      using LoadT       = EMT::LoadRL<ScalarT, IdxT>;
      using RealT       = typename LoadT::RealT;
      using BusT        = typename LoadT::BusT;
      using DataT       = typename LoadT::ModelDataT;
      using SourceT     = EMT::VoltageSource<ScalarT, IdxT, 3>;
      using SourceDataT = typename SourceT::ModelDataT;

      TestOutcome parser()
      {
        TestStatus success = true;

        nlohmann::json j = {
            {"class", "LoadRL"},
            {"id", "load1"},
            {"params",
             {{"Ra", 1.0},
              {"Rb", 2.0},
              {"Rc", 3.0},
              {"La", 0.1},
              {"Lb", 0.2},
              {"Lc", 0.3},
              {"Iinj", 5.0},
              {"theta", 0.25}}}};

        auto data = j.get<DataT>();

        success *= data.device_class == "LoadRL";
        success *= data.disambiguation_string == "load1";
        success *= isEqual(data.R[1], RealT{2.0});
        success *= isEqual(data.L[2], RealT{0.3});
        success *= isEqual(data.Iinj, RealT{5.0});
        success *= isEqual(data.theta, RealT{0.25});

        return success.report(__func__);
      }

      TestOutcome parserRejectsInvalidParameters()
      {
        TestStatus success = true;

        nlohmann::json base = {
            {"class", "LoadRL"},
            {"id", "bad"},
            {"params",
             {{"Ra", 1.0},
              {"Rb", 2.0},
              {"Rc", 3.0},
              {"La", 0.1},
              {"Lb", 0.2},
              {"Lc", 0.3}}}};

        auto bad_r              = base;
        bad_r["params"]["Rb"]   = -1.0;
        auto bad_l              = base;
        bad_l["params"]["Lc"]   = 0.0;
        auto bad_i              = base;
        bad_i["params"]["Iinj"] = -1.0;

        success *= throws<std::invalid_argument>(
            [&bad_r]()
            { bad_r.get<DataT>(); });
        success *= throws<std::invalid_argument>(
            [&bad_l]()
            { bad_l.get<DataT>(); });
        success *= throws<std::invalid_argument>(
            [&bad_i]()
            { bad_i.get<DataT>(); });

        return success.report(__func__);
      }

      TestOutcome verifyRejectsInvalidConfiguration()
      {
        TestStatus success = true;

        BusT  bus2(2, std::vector<RealT>{1.0, 2.0});
        BusT  bus3(3, std::vector<RealT>{1.0, 2.0, 3.0});
        DataT data = makeData();

        LoadT missing_bus(nullptr, data);
        success *= missing_bus.verify() > 0;

        LoadT mismatched_bus(&bus2, data);
        success *= mismatched_bus.verify() > 0;

        data.L[0] = 0.0;
        LoadT bad_l(&bus3, data);
        success *= bad_l.verify() > 0;

        return success.report(__func__);
      }

      TestOutcome initializeFromPhasor()
      {
        TestStatus success = true;

        BusT  bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        DataT data = makeData();
        data.Iinj  = 5.0;
        data.theta = 0.0;
        LoadT load(&bus, data);

        success *= bus.allocate() == 0;
        success *= load.allocate() == 0;
        success *= load.initialize() == 0;

        const RealT sqrt2  = std::sqrt(RealT{2.0});
        const RealT pi     = std::acos(RealT{-1.0});
        success           *= isEqual(load.I(0), sqrt2 * data.Iinj);
        success           *= isEqual(load.I(1), sqrt2 * data.Iinj * std::cos(-RealT{2.0} * pi / RealT{3.0}));
        success           *= isEqual(load.I(2), sqrt2 * data.Iinj * std::cos(RealT{2.0} * pi / RealT{3.0}));
        success           *= isEqual(load.yp()[0], ScalarT{0.0});
        success           *= isEqual(load.yp()[1], ScalarT{0.0});
        success           *= isEqual(load.yp()[2], ScalarT{0.0});

        return success.report(__func__);
      }

      TestOutcome residualAndBusInjection()
      {
        TestStatus success = true;

        BusT  bus(1, std::vector<RealT>{10.0, 20.0, -30.0});
        DataT data = makeData();
        LoadT load(&bus, data);

        success *= bus.allocate() == 0;
        success *= bus.initialize() == 0;
        success *= load.allocate() == 0;
        success *= load.initialize() == 0;

        load.y()[0]  = 4.0;
        load.y()[1]  = -5.0;
        load.y()[2]  = 6.0;
        load.yp()[0] = 0.7;
        load.yp()[1] = -0.8;
        load.yp()[2] = 0.9;

        success *= bus.evaluateResidual() == 0;
        success *= load.evaluateResidual() == 0;

        for (std::size_t n = 0; n < 3; ++n)
        {
          const auto idx  = static_cast<IdxT>(n);
          success        *= isEqual(load.getResidual()[n],
                             data.R[n] * load.y()[n] + data.L[n] * load.yp()[n] + bus.V(idx));
          success        *= isEqual(bus.I(idx), load.y()[n]);
        }

        return success.report(__func__);
      }

      TestOutcome tagsAndTolerance()
      {
        TestStatus success = true;

        BusT  bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        LoadT load(&bus, makeData());

        success *= load.allocate() == 0;
        success *= load.tagDifferentiable() == 0;
        success *= load.setAbsoluteTolerance(1.0e-6) == 0;

        for (std::size_t n = 0; n < 3; ++n)
        {
          success *= isEqual(load.tag()[n], ScalarT{1.0});
          success *= isEqual(load.absoluteTolerance()[n], ScalarT{1.0e-6});
        }

        return success.report(__func__);
      }

      TestOutcome systemModel()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        BusT                                       bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        LoadT                                      load(&bus, makeData());

        system.addBus(&bus);
        system.addComponent(&load);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.size() == bus.size() + load.size();

        return success.report(__func__);
      }

      TestOutcome sourceLoadSystemModel()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        BusT                                       bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        SourceT                                    source(&bus, makeSourceData());
        LoadT                                      load(&bus, makeData());

        system.addBus(&bus);
        system.addComponent(&source);
        system.addComponent(&load);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.size() == bus.size() + source.size() + load.size();

        const auto source_data = makeSourceData();
        for (std::size_t n = 0; n < 3; ++n)
        {
          const auto idx              = static_cast<IdxT>(n);
          const auto expected_source  = -source_data.G[n] * bus.V(idx);
          success                    *= isEqual(bus.I(idx), expected_source + load.I(idx));
        }

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        BusT  bus(1, std::vector<RealT>{10.0, 20.0, 30.0});
        DataT data = makeData();
        data.R     = {1.0, 2.0, 3.0};
        data.L     = {0.5, 0.25, 0.125};
        LoadT load(&bus, data);

        success *= bus.allocate() == 0;
        success *= load.allocate() == 0;

        for (IdxT n = 0; n < bus.size(); ++n)
        {
          bus.setVariableIndex(n, static_cast<IdxT>(100 + n));
          bus.setResidualIndex(n, static_cast<IdxT>(200 + n));
        }

        for (IdxT n = 0; n < load.size(); ++n)
        {
          load.setVariableIndex(n, static_cast<IdxT>(10 + n));
          load.setResidualIndex(n, static_cast<IdxT>(20 + n));
        }

        load.updateTime(0.0, 4.0);
        success *= load.evaluateJacobian() == 0;

        load.getJacobian().deduplicate();
        auto actual = GridKit::Testing::MapFromCOO(load.getJacobian());

        for (std::size_t n = 0; n < 3; ++n)
        {
          success *= isEqual(actual[20 + n][static_cast<IdxT>(10 + n)],
                             data.R[n] + RealT{4.0} * data.L[n]);
          success *= isEqual(actual[20 + n][static_cast<IdxT>(100 + n)], RealT{1.0});
          success *= isEqual(actual[200 + n][static_cast<IdxT>(10 + n)], RealT{1.0});
        }

        return success.report(__func__);
      }
#endif

    private:
      static DataT makeData()
      {
        DataT data;
        data.R     = {1.0, 2.0, 3.0};
        data.L     = {0.1, 0.2, 0.3};
        data.Iinj  = 0.0;
        data.theta = 0.0;
        return data;
      }

      static SourceDataT makeSourceData()
      {
        SourceDataT data;
        data.E     = {0.0, 0.0, 0.0};
        data.phi   = {0.0, 0.0, 0.0};
        data.omega = 1.0;
        data.G     = {0.1, 0.2, 0.3};
        return data;
      }
    };
  } // namespace Testing
} // namespace GridKit
