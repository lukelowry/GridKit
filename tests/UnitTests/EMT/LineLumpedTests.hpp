#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumped.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedDataJSONParser.hpp>
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
    class EMTLineLumpedTests
    {
    public:
      using LineT = EMT::LineLumped<ScalarT, IdxT, 2>;
      using RealT = typename LineT::RealT;
      using BusT  = typename LineT::BusT;
      using DataT = typename LineT::ModelDataT;

      TestOutcome allocationAndTags()
      {
        TestStatus success = true;

        BusT  bus1(1, std::vector<RealT>{1.0, 2.0});
        BusT  bus2(2, std::vector<RealT>{3.0, 4.0});
        LineT line(&bus1, &bus2, makeData<1>());

        success *= bus1.allocate() == 0;
        success *= bus2.allocate() == 0;
        success *= line.allocate() == 0;
        success *= line.verify() == 0;
        success *= line.size() == 20;
        success *= line.tagDifferentiable() == 0;

        success *= isEqual(line.tag()[0], ScalarT{1.0});
        success *= isEqual(line.tag()[1], ScalarT{1.0});

        const std::array<std::size_t, 3> output_offsets{6, 12, 18};
        for (auto output : output_offsets)
        {
          success *= isEqual(line.tag()[output], ScalarT{0.0});
          success *= isEqual(line.tag()[output + 1], ScalarT{0.0});
        }

        return success.report(__func__);
      }

      TestOutcome parserConstantParameters()
      {
        TestStatus success = true;

        nlohmann::json j = {
            {"class", "LineLumped"},
            {"id", "line1"},
            {"params",
             {{"dx", 2.0},
              {"i0", {1.0, -2.0}},
              {"Rp", {{1.0, 2.0}, {3.0, 4.0}}},
              {"Lp", {{0.1, 0.2}, {0.3, 0.4}}},
              {"Gp", {{5.0, 6.0}, {7.0, 8.0}}},
              {"Cp", {{0.5, 0.6}, {0.7, 0.8}}}}}};

        auto data = j.get<DataT>();

        success *= data.device_class == "LineLumped";
        success *= data.disambiguation_string == "line1";
        success *= isEqual(data.dx, RealT{2.0});
        success *= isEqual(data.i0[0], RealT{1.0});
        success *= isEqual(data.i0[1], RealT{-2.0});
        success *= isEqual(data.Zp.D(1, 0), RealT{3.0});
        success *= isEqual(data.Zp.E(0, 1), RealT{0.2});
        success *= isEqual(data.Yp.D(1, 1), RealT{8.0});
        success *= isEqual(data.Yp.E(0, 0), RealT{0.5});

        return success.report(__func__);
      }

      TestOutcome parserFittedParameters()
      {
        TestStatus success = true;

        nlohmann::json vf = {
            {"params",
             {{"D", {{1.0, 0.0}, {0.0, 1.0}}},
              {"E", {{0.1, 0.0}, {0.0, 0.1}}},
              {"poles", nlohmann::json::array()},
              {"A", nlohmann::json::array()},
              {"B", nlohmann::json::array()}}}};

        nlohmann::json j = {
            {"class", "LineLumped"},
            {"id", "line2"},
            {"params", {{"dx", 10.0}, {"Zp", vf}, {"Yp", vf}}}};

        auto data = j.get<DataT>();

        success *= data.Zp.device_class == "VectorFit";
        success *= data.Yp.device_class == "VectorFit";
        success *= data.Zp.disambiguation_string == "line2.Zp";
        success *= isEqual(data.Yp.E(1, 1), RealT{0.1});

        return success.report(__func__);
      }

      TestOutcome parserRejectsInvalidParameters()
      {
        TestStatus success = true;

        nlohmann::json bad_dx = {
            {"class", "LineLumped"},
            {"id", "bad"},
            {"params",
             {{"dx", 0.0},
              {"Rp", {{1.0, 0.0}, {0.0, 1.0}}},
              {"Lp", {{0.0, 0.0}, {0.0, 0.0}}},
              {"Gp", {{0.0, 0.0}, {0.0, 0.0}}},
              {"Cp", {{0.0, 0.0}, {0.0, 0.0}}}}}};

        nlohmann::json mixed  = bad_dx;
        mixed["params"]["dx"] = 1.0;
        mixed["params"]["Zp"] = {{"params", {{"D", {{1.0, 0.0}, {0.0, 1.0}}}, {"E", {{0.0, 0.0}, {0.0, 0.0}}}, {"poles", nlohmann::json::array()}, {"A", nlohmann::json::array()}, {"B", nlohmann::json::array()}}}};
        mixed["params"]["Yp"] = mixed["params"]["Zp"];

        success *= throws<std::invalid_argument>(
            [&bad_dx]()
            { bad_dx.get<DataT>(); });
        success *= throws<std::invalid_argument>(
            [&mixed]()
            { mixed.get<DataT>(); });

        return success.report(__func__);
      }

      TestOutcome verifyRejectsInvalidConfiguration()
      {
        TestStatus success = true;

        BusT  bus2(2, std::vector<RealT>{3.0, 4.0});
        BusT  bus3(3, std::vector<RealT>{1.0, 2.0, 3.0});
        DataT data = makeData<0>();

        LineT missing_bus(nullptr, &bus2, data);
        success *= missing_bus.verify() > 0;

        LineT mismatched_bus(&bus3, &bus2, data);
        success *= bus3.allocate() == 0;
        success *= bus2.allocate() == 0;
        success *= mismatched_bus.verify() > 0;

        data.dx = 0.0;
        LineT bad_dx(&bus2, &bus2, data);
        success *= bad_dx.verify() > 0;

        return success.report(__func__);
      }

      TestOutcome initializeConstantOperators()
      {
        TestStatus success = true;

        BusT  bus1(1, std::vector<RealT>{10.0, 20.0});
        BusT  bus2(2, std::vector<RealT>{30.0, 40.0});
        DataT data = makeData<0>();
        data.dx    = 2.0;
        data.i0    = {1.0, -2.0};

        LineT line(&bus1, &bus2, data);

        success *= bus1.allocate() == 0;
        success *= bus2.allocate() == 0;
        success *= bus1.initialize() == 0;
        success *= bus2.initialize() == 0;
        success *= line.allocate() == 0;
        success *= line.initialize() == 0;

        success *= isEqual(line.y()[0], ScalarT{1.0});
        success *= isEqual(line.y()[1], ScalarT{-2.0});

        success *= isEqual(line.y()[2], ScalarT{-4.0});
        success *= isEqual(line.y()[3], ScalarT{-6.0});
        success *= isEqual(line.y()[4], ScalarT{20.0});
        success *= isEqual(line.y()[5], ScalarT{27.5});
        success *= isEqual(line.y()[6], ScalarT{45.0});
        success *= isEqual(line.y()[7], ScalarT{62.5});

        return success.report(__func__);
      }

      TestOutcome residualAndTerminalCurrents()
      {
        TestStatus success = true;

        BusT  bus1(1, std::vector<RealT>{10.0, 20.0});
        BusT  bus2(2, std::vector<RealT>{7.0, 25.0});
        LineT line(&bus1, &bus2, makeZeroData());

        success *= bus1.allocate() == 0;
        success *= bus2.allocate() == 0;
        success *= bus1.initialize() == 0;
        success *= bus2.initialize() == 0;
        success *= line.allocate() == 0;
        success *= line.initialize() == 0;

        line.y()[0] = 1.0;
        line.y()[1] = -2.0;
        line.y()[2] = 0.5;
        line.y()[3] = -1.5;
        line.y()[4] = 2.0;
        line.y()[5] = 3.0;
        line.y()[6] = -4.0;
        line.y()[7] = 5.0;

        success *= bus1.evaluateResidual() == 0;
        success *= bus2.evaluateResidual() == 0;
        success *= line.evaluateResidual() == 0;

        success *= isEqual(line.getResidual()[0], ScalarT{-2.5});
        success *= isEqual(line.getResidual()[1], ScalarT{3.5});
        success *= isEqual(bus1.I(0), ScalarT{-3.0});
        success *= isEqual(bus1.I(1), ScalarT{-1.0});
        success *= isEqual(bus2.I(0), ScalarT{5.0});
        success *= isEqual(bus2.I(1), ScalarT{-7.0});

        return success.report(__func__);
      }

      TestOutcome systemModel()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        BusT                                       bus1(1, std::vector<RealT>{10.0, 20.0});
        BusT                                       bus2(2, std::vector<RealT>{7.0, 25.0});
        LineT                                      line(&bus1, &bus2, makeZeroData());

        system.addBus(&bus1);
        system.addBus(&bus2);
        system.addComponent(&line);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= system.evaluateResidual() == 0;
        success *= system.size() == bus1.size() + bus2.size() + line.size();

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        BusT  bus1(1, std::vector<RealT>{10.0, 20.0});
        BusT  bus2(2, std::vector<RealT>{7.0, 25.0});
        DataT data = makeData<0>();
        data.dx    = 2.0;
        LineT line(&bus1, &bus2, data);

        success *= bus1.allocate() == 0;
        success *= bus2.allocate() == 0;
        success *= line.allocate() == 0;

        for (IdxT n = 0; n < bus1.size(); ++n)
        {
          bus1.setVariableIndex(n, static_cast<IdxT>(100 + n));
          bus1.setResidualIndex(n, static_cast<IdxT>(200 + n));
          bus2.setVariableIndex(n, static_cast<IdxT>(110 + n));
          bus2.setResidualIndex(n, static_cast<IdxT>(210 + n));
        }

        for (IdxT j = 0; j < line.size(); ++j)
        {
          line.setVariableIndex(j, static_cast<IdxT>(10 + j));
          line.setResidualIndex(j, static_cast<IdxT>(20 + j));
        }

        line.updateTime(0.0, 4.0);
        success *= line.evaluateJacobian() == 0;

        line.getJacobian().deduplicate();
        auto actual = GridKit::Testing::MapFromCOO(line.getJacobian());

        std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expected(actual.size());

        expected[20][12]  = 1.0;
        expected[20][110] = 1.0;
        expected[20][100] = -1.0;
        expected[21][13]  = 1.0;
        expected[21][111] = 1.0;
        expected[21][101] = -1.0;

        expected[22][12] = 1.0;
        expected[22][10] = -2.8;
        expected[22][11] = -4.2;
        expected[23][13] = 1.0;
        expected[23][10] = -5.2;
        expected[23][11] = -6.6;

        expected[24][14]  = 1.0;
        expected[24][100] = -1.3;
        expected[24][101] = -1.95;
        expected[25][15]  = 1.0;
        expected[25][100] = -1.95;
        expected[25][101] = -2.6;

        expected[26][16]  = 1.0;
        expected[26][110] = -1.3;
        expected[26][111] = -1.95;
        expected[27][17]  = 1.0;
        expected[27][110] = -1.95;
        expected[27][111] = -2.6;

        expected[200][10] = -1.0;
        expected[200][14] = -1.0;
        expected[201][11] = -1.0;
        expected[201][15] = -1.0;
        expected[210][10] = 1.0;
        expected[210][16] = -1.0;
        expected[211][11] = 1.0;
        expected[211][17] = -1.0;

        for (std::size_t row = 0; row < expected.size(); ++row)
        {
          if (!expected[row].empty())
          {
            success *= isEqual(actual[row], expected[row]);
          }
        }

        return success.report(__func__);
      }
#endif

    private:
      static DataT makeZeroData()
      {
        DataT data;
        data.dx = 1.0;
        return data;
      }

      template <std::size_t Q>
      static DataT makeData()
      {
        DataT data;
        data.dx = 1.0;

        data.Zp.poles.resize(Q);
        data.Zp.A.resize(Q);
        data.Zp.B.resize(Q);
        data.Yp.poles.resize(Q);
        data.Yp.A.resize(Q);
        data.Yp.B.resize(Q);

        for (std::size_t r = 0; r < 2; ++r)
        {
          for (std::size_t c = 0; c < 2; ++c)
          {
            const RealT rr = static_cast<RealT>(r);
            const RealT cc = static_cast<RealT>(c);

            data.Zp.D(r, c) = RealT{1.0} + rr + RealT{0.5} * cc;
            data.Zp.E(r, c) = RealT{0.1} + RealT{0.05} * rr + RealT{0.05} * cc;
            data.Yp.D(r, c) = RealT{0.5} + RealT{0.25} * rr + RealT{0.25} * cc;
            data.Yp.E(r, c) = RealT{0.2} + RealT{0.1} * rr + RealT{0.1} * cc;
          }
        }

        for (std::size_t q = 0; q < Q; ++q)
        {
          const RealT qq = static_cast<RealT>(q);

          data.Zp.poles[q] = {RealT{-2.0} - qq, RealT{3.0} + qq};
          data.Yp.poles[q] = {RealT{-4.0} - qq, RealT{5.0} + qq};

          for (std::size_t r = 0; r < 2; ++r)
          {
            for (std::size_t c = 0; c < 2; ++c)
            {
              data.Zp.A[q](r, c) = RealT{0.1} + qq + static_cast<RealT>(r + c);
              data.Zp.B[q](r, c) = RealT{0.2} + qq + static_cast<RealT>(r + c);
              data.Yp.A[q](r, c) = RealT{0.3} + qq + static_cast<RealT>(r + c);
              data.Yp.B[q](r, c) = RealT{0.4} + qq + static_cast<RealT>(r + c);
            }
          }
        }

        return data;
      }
    };
  } // namespace Testing
} // namespace GridKit
