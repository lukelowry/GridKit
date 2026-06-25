#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFit.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitDataJSONParser.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/Testing.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/Utilities/MapFromCOO.hpp>
#endif

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTVectorFitTests
    {
    public:
      using RealT   = typename EMT::VectorFit<ScalarT, IdxT, 1, 1>::RealT;
      using SignalT = PhasorDynamics::SignalNode<ScalarT, IdxT>;

      TestOutcome allocationAndSignals()
      {
        TestStatus success = true;

        constexpr std::size_t N     = 2;
        constexpr std::size_t K     = 3;
        constexpr std::size_t Q     = 1;
        constexpr std::size_t Y_OUT = 2 * Q * K;

        EMT::VectorFit<ScalarT, IdxT, N, K> model(makeData<N, K, Q>());

        std::array<ScalarT, K> u{1.0, 2.0, 3.0};
        std::array<ScalarT, K> up{0.1, 0.2, 0.3};
        std::array<IdxT, K>    input_indices{10, 11, 12};
        std::array<SignalT, K> input_nodes{};
        std::array<SignalT, N> output_nodes{};

        connectInputs(model, u, up, input_indices, input_nodes);
        assignOutputs(model, output_nodes);

        success *= model.getSignals().template externalPortCount<EMT::VectorFitExternalVariables::INPUT>() == K;
        success *= model.getSignals().template internalPortCount<EMT::VectorFitInternalVariables::Y_OUT>() == N;
        success *= model.verify() == 0;
        success *= model.allocate() == 0;
        success *= model.size() == static_cast<IdxT>(Y_OUT + N);

        for (std::size_t n = 0; n < N; ++n)
        {
          success *= output_nodes[n].linked();
          success *= output_nodes[n].getVariableIndex() == static_cast<IdxT>(Y_OUT + n);
        }

        return success.report(__func__);
      }

      TestOutcome q0Residual()
      {
        TestStatus success = true;

        constexpr std::size_t N = 2;
        constexpr std::size_t K = 3;

        auto                                data = makeData<N, K, 0>();
        EMT::VectorFit<ScalarT, IdxT, N, K> model(data);

        std::array<ScalarT, K> u{1.0, -2.0, 0.5};
        std::array<ScalarT, K> up{0.25, 1.5, -1.0};
        std::array<IdxT, K>    input_indices{10, 11, 12};
        std::array<SignalT, K> input_nodes{};
        std::array<SignalT, N> output_nodes{};

        connectInputs(model, u, up, input_indices, input_nodes);
        assignOutputs(model, output_nodes);

        success *= model.allocate() == 0;

        model.y()[0] = 3.0;
        model.y()[1] = -4.0;

        success *= model.evaluateResidual() == 0;

        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT expected = model.y()[n];
          for (std::size_t k = 0; k < K; ++k)
          {
            expected -= data.D(n, k) * u[k] + data.E(n, k) * up[k];
          }
          success *= isEqual(model.getResidual()[n], expected);
        }

        return success.report(__func__);
      }

      TestOutcome poleResidualLayout()
      {
        TestStatus success = true;

        constexpr std::size_t N     = 2;
        constexpr std::size_t K     = 2;
        constexpr std::size_t Q     = 1;
        constexpr std::size_t Y_OUT = 2 * Q * K;

        auto                                data = makeData<N, K, Q>();
        EMT::VectorFit<ScalarT, IdxT, N, K> model(data);

        std::array<ScalarT, Y_OUT + N> y{1.0, -2.0, 0.5, -0.25, 4.0, -3.0};
        std::array<ScalarT, Y_OUT + N> yp{0.25, -0.5, 1.5, -1.0, 0.0, 0.0};
        std::array<ScalarT, K>         ws{2.0, -1.0};
        std::array<ScalarT, K>         wsp{0.75, -0.5};
        std::array<ScalarT, 1>         wb{};
        std::array<ScalarT, Y_OUT + N> f{};

        success *= model.evaluateInternalResidual(y.data(), yp.data(), wb.data(), ws.data(), wsp.data(), f.data()) == 0;

        const RealT a     = data.poles[0][0];
        const RealT omega = data.poles[0][1];

        for (std::size_t k = 0; k < K; ++k)
        {
          success *= isEqual(f[k],
                             -yp[k] + a * y[k] - omega * y[K + k] + ws[k]);
          success *= isEqual(f[K + k],
                             -yp[K + k] + omega * y[k] + a * y[K + k]);
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT expected = y[Y_OUT + n];
          for (std::size_t k = 0; k < K; ++k)
          {
            expected -= data.D(n, k) * ws[k];
            expected -= data.E(n, k) * wsp[k];
            expected -= data.A[0](n, k) * y[k];
            expected += data.B[0](n, k) * y[K + k];
          }
          success *= isEqual(f[Y_OUT + n], expected);
        }

        return success.report(__func__);
      }

      TestOutcome initializeAffineInput()
      {
        TestStatus success = true;

        constexpr std::size_t N     = 2;
        constexpr std::size_t K     = 2;
        constexpr std::size_t Q     = 1;
        constexpr std::size_t Y_OUT = 2 * Q * K;

        auto                                data = makeData<N, K, Q>();
        EMT::VectorFit<ScalarT, IdxT, N, K> model(data);

        std::array<ScalarT, K> u{1.0, -2.0};
        std::array<ScalarT, K> up{0.5, -0.25};
        std::array<IdxT, K>    input_indices{10, 11};
        std::array<SignalT, K> input_nodes{};
        std::array<SignalT, N> output_nodes{};

        connectInputs(model, u, up, input_indices, input_nodes);
        assignOutputs(model, output_nodes);

        success *= model.allocate() == 0;
        success *= model.initialize() == 0;
        success *= model.evaluateResidual() == 0;

        const RealT a     = data.poles[0][0];
        const RealT omega = data.poles[0][1];
        const RealT den   = a * a + omega * omega;
        const RealT den2  = den * den;
        const RealT tol   = 1.0e-12;

        for (std::size_t k = 0; k < K; ++k)
        {
          const ScalarT w =
              -a / den * u[k] - (a * a - omega * omega) / den2 * up[k];
          const ScalarT v =
              omega / den * u[k] + 2.0 * a * omega / den2 * up[k];

          success *= isEqual(model.y()[k], w, tol);
          success *= isEqual(model.y()[K + k], v, tol);
          success *= isEqual(model.yp()[k], a * w - omega * v + u[k], tol);
          success *= isEqual(model.yp()[K + k], omega * w + a * v, tol);
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT expected = 0.0;
          for (std::size_t k = 0; k < K; ++k)
          {
            expected += data.D(n, k) * u[k];
            expected += data.E(n, k) * up[k];
            expected += data.A[0](n, k) * model.y()[k];
            expected -= data.B[0](n, k) * model.y()[K + k];
          }
          success *= isEqual(model.y()[Y_OUT + n], expected, tol);
          success *= isEqual(model.getResidual()[Y_OUT + n], ScalarT{0.0}, tol);
        }

        return success.report(__func__);
      }

      TestOutcome differentiabilityTags()
      {
        TestStatus success = true;

        constexpr std::size_t N     = 2;
        constexpr std::size_t K     = 2;
        constexpr std::size_t Q     = 1;
        constexpr std::size_t Y_OUT = 2 * Q * K;

        EMT::VectorFit<ScalarT, IdxT, N, K> model(makeData<N, K, Q>());

        success *= model.allocate() == 0;
        success *= model.tagDifferentiable() == 0;

        for (std::size_t j = 0; j < Y_OUT; ++j)
        {
          success *= isEqual(model.tag()[j], ScalarT{1.0});
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          success *= isEqual(model.tag()[Y_OUT + n], ScalarT{0.0});
        }

        return success.report(__func__);
      }

      TestOutcome parser()
      {
        TestStatus success = true;

        nlohmann::json j = {
            {"class", "VectorFit"},
            {"id", "vf1"},
            {"params",
             {{"D", {{1.0, 2.0}, {3.0, 4.0}}},
              {"E", {{0.1, 0.2}, {0.3, 0.4}}},
              {"poles", {{-2.0, 3.0}}},
              {"A", {{{5.0, 6.0}, {7.0, 8.0}}}},
              {"B", {{{0.5, 0.6}, {0.7, 0.8}}}}}}};

        auto data = j.get<EMT::VectorFitData<RealT, IdxT, 2, 2>>();

        success *= data.device_class == "VectorFit";
        success *= data.disambiguation_string == "vf1";
        success *= data.poles.size() == 1;
        success *= data.A.size() == 1;
        success *= data.B.size() == 1;
        success *= isEqual(data.D(1, 0), RealT{3.0});
        success *= isEqual(data.E(0, 1), RealT{0.2});
        success *= isEqual(data.poles[0][0], RealT{-2.0});
        success *= isEqual(data.poles[0][1], RealT{3.0});
        success *= isEqual(data.A[0](1, 1), RealT{8.0});
        success *= isEqual(data.B[0](0, 0), RealT{0.5});

        return success.report(__func__);
      }

      TestOutcome verifyRejectsInvalidSignals()
      {
        TestStatus success = true;

        {
          EMT::VectorFit<ScalarT, IdxT, 1, 1> model(makeData<1, 1, 0>());
          success *= model.verify() > 0;
        }

        {
          EMT::VectorFit<ScalarT, IdxT, 1, 1> model(makeData<1, 1, 0>());
          ScalarT                             u{1.0};
          ScalarT                             up{0.0};
          IdxT                                index{10};
          SignalT                             input_node;

          input_node.set(&u, &up, &index, true);
          model.getSignals().template attachSignalNode<EMT::VectorFitExternalVariables::INPUT>(0, 0, &input_node);

          success *= model.verify() > 0;
        }

        {
          EMT::VectorFit<ScalarT, IdxT, 1, 1> model(makeData<1, 1, 0>());
          ScalarT                             u{1.0};
          IdxT                                index{10};
          SignalT                             input_node;
          SignalT                             output_node;

          input_node.set(&u, &index);
          model.getSignals().template attachSignalNode<EMT::VectorFitExternalVariables::INPUT>(0, 0, &input_node);
          model.getSignals().template assignSignalNode<EMT::VectorFitInternalVariables::Y_OUT>(0, 0, &output_node);

          success *= model.verify() > 0;
        }

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        constexpr std::size_t N     = 2;
        constexpr std::size_t K     = 2;
        constexpr std::size_t Q     = 1;
        constexpr std::size_t Y_OUT = 2 * Q * K;

        auto                                data = makeData<N, K, Q>();
        EMT::VectorFit<ScalarT, IdxT, N, K> model(data);

        std::array<ScalarT, K> u{1.0, -2.0};
        std::array<ScalarT, K> up{0.5, -0.25};
        std::array<IdxT, K>    input_indices{30, 31};
        std::array<SignalT, K> input_nodes{};
        std::array<SignalT, N> output_nodes{};

        connectInputs(model, u, up, input_indices, input_nodes);
        assignOutputs(model, output_nodes);

        success *= model.allocate() == 0;

        for (std::size_t j = 0; j < Y_OUT + N; ++j)
        {
          model.setVariableIndex(static_cast<IdxT>(j), static_cast<IdxT>(10 + j));
          model.setResidualIndex(static_cast<IdxT>(j), static_cast<IdxT>(20 + j));
        }

        model.updateTime(0.0, 4.0);
        success *= model.evaluateJacobian() == 0;

        model.getJacobian().deduplicate();
        auto actual = GridKit::Testing::MapFromCOO(model.getJacobian());

        std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expected(20 + Y_OUT + N);

        const RealT a     = data.poles[0][0];
        const RealT omega = data.poles[0][1];

        for (std::size_t k = 0; k < K; ++k)
        {
          expected[20 + k][10 + k]           = a - 4.0;
          expected[20 + k][10 + K + k]       = -omega;
          expected[20 + k][input_indices[k]] = 1.0;

          expected[20 + K + k][10 + k]     = omega;
          expected[20 + K + k][10 + K + k] = a - 4.0;
        }

        for (std::size_t n = 0; n < N; ++n)
        {
          expected[20 + Y_OUT + n][10 + Y_OUT + n] = 1.0;

          for (std::size_t k = 0; k < K; ++k)
          {
            expected[20 + Y_OUT + n][10 + k]     = -data.A[0](n, k);
            expected[20 + Y_OUT + n][10 + K + k] = data.B[0](n, k);
            expected[20 + Y_OUT + n][input_indices[k]] =
                -data.D(n, k) - 4.0 * data.E(n, k);
          }
        }

        for (std::size_t row = 20; row < 20 + Y_OUT + N; ++row)
        {
          success *= isEqual(actual[row], expected[row]);
        }

        return success.report(__func__);
      }
#endif

    private:
      template <std::size_t N, std::size_t K, std::size_t Q>
      EMT::VectorFitData<RealT, IdxT, N, K> makeData()
      {
        EMT::VectorFitData<RealT, IdxT, N, K> data;
        data.poles.resize(Q);
        data.A.resize(Q);
        data.B.resize(Q);

        for (std::size_t n = 0; n < N; ++n)
        {
          for (std::size_t k = 0; k < K; ++k)
          {
            const auto nr = static_cast<RealT>(n);
            const auto kr = static_cast<RealT>(k);

            data.D(n, k) = static_cast<RealT>(1.0) + nr + static_cast<RealT>(2.0) * kr;
            data.E(n, k) = static_cast<RealT>(0.25) + nr + kr;
          }
        }

        for (std::size_t q = 0; q < Q; ++q)
        {
          const auto qr = static_cast<RealT>(q);

          data.poles[q][0] = static_cast<RealT>(-2.0) - qr;
          data.poles[q][1] = static_cast<RealT>(3.0) + qr;

          for (std::size_t n = 0; n < N; ++n)
          {
            for (std::size_t k = 0; k < K; ++k)
            {
              const auto nr = static_cast<RealT>(n);
              const auto kr = static_cast<RealT>(k);

              data.A[q](n, k) = static_cast<RealT>(2.0) + qr + nr + static_cast<RealT>(3.0) * kr;
              data.B[q](n, k) = static_cast<RealT>(0.5) + qr + nr + kr;
            }
          }
        }

        return data;
      }

      template <std::size_t N, std::size_t K>
      void connectInputs(EMT::VectorFit<ScalarT, IdxT, N, K>& model,
                         std::array<ScalarT, K>&              u,
                         std::array<ScalarT, K>&              up,
                         std::array<IdxT, K>&                 indices,
                         std::array<SignalT, K>&              nodes)
      {
        for (std::size_t k = 0; k < K; ++k)
        {
          nodes[k].set(&u[k], &up[k], &indices[k], true);
          model.getSignals().template attachSignalNode<EMT::VectorFitExternalVariables::INPUT>(k, 0, &nodes[k]);
        }
      }

      template <std::size_t N, std::size_t K>
      void assignOutputs(EMT::VectorFit<ScalarT, IdxT, N, K>& model,
                         std::array<SignalT, N>&              nodes)
      {
        for (std::size_t n = 0; n < N; ++n)
        {
          model.getSignals().template assignSignalNode<EMT::VectorFitInternalVariables::Y_OUT>(n, 0, &nodes[n]);
        }
      }
    };
  } // namespace Testing
} // namespace GridKit
