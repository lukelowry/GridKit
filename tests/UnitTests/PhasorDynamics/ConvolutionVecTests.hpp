#pragma once

#include <iostream>
#include <limits>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVec.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVecData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class ConvolutionVecTests
    {
    public:
      using RealT = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;

      ConvolutionVecTests()  = default;
      ~ConvolutionVecTests() = default;

      TestOutcome constructor()
      {
        TestStatus success = true;

        auto nodes = makeSignalNodes();
        auto data  = makeTestData();

        auto* convolution =
            new PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT>(nodes.input_ptrs, nodes.output_ptrs, data);

        success *= (convolution != nullptr);
        success *= (convolution->dimension() == data.dimension);
        success *= (convolution->modeCount() == data.p.size());

        delete convolution;

        PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> data_only(data);
        success *= (data_only.dimension() == data.dimension);
        success *= (data_only.modeCount() == data.p.size());
        success *= (data_only.verify() > 0);
        success *= (data_only.attachInputSignals(nodes.input_ptrs) == 0);
        success *= (data_only.assignOutputSignals(nodes.output_ptrs) == 0);
        success *= (data_only.verify() == 0);

        auto late_nodes = makeSignalNodes();

        PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> late_outputs(data);
        success *= (late_outputs.attachInputSignals(late_nodes.input_ptrs) == 0);
        late_outputs.allocate();
        success *= (late_outputs.assignOutputSignals(late_nodes.output_ptrs) == 0);
        success *= late_nodes.outputs[0].linked();
        success *= late_nodes.outputs[1].linked();

        return success.report(__func__);
      }

      TestOutcome verifyFailures()
      {
        TestStatus success = true;

        auto nodes = makeSignalNodes();
        auto data  = makeTestData();

        {
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, data);
          success *= (convolution.verify() == 0);
        }

        {
          auto bad_data      = data;
          bad_data.dimension = 0;
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, bad_data);
          success *= (convolution.verify() > 0);
        }

        {
          auto bad_data = data;
          bad_data.d.pop_back();
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, bad_data);
          success *= (convolution.verify() > 0);
        }

        {
          auto bad_data = data;
          bad_data.b.pop_back();
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, bad_data);
          success *= (convolution.verify() > 0);
        }

        {
          auto bad_data = data;
          bad_data.p[0] = 0.0;
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, bad_data);
          success *= (convolution.verify() > 0);
        }

        {
          auto input_ptrs = nodes.input_ptrs;
          input_ptrs.pop_back();
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(input_ptrs, nodes.output_ptrs, data);
          success *= (convolution.verify() > 0);
        }

        {
          auto output_ptrs = nodes.output_ptrs;
          output_ptrs[1]   = nullptr;
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, output_ptrs, data);
          success *= (convolution.verify() > 0);
        }

        {
          auto                                                       unlinked_nodes = makeSignalNodes(false);
          PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(
              unlinked_nodes.input_ptrs, unlinked_nodes.output_ptrs, data);
          success *= (convolution.verify() > 0);
        }

        return success.report(__func__);
      }

      TestOutcome zeroInitialResidual()
      {
        TestStatus success = true;

        auto data  = makeTestData();
        auto nodes = makeSignalNodes();
        for (size_t i = 0; i < data.dimension; ++i)
        {
          nodes.input_values[i] = data.u0[i];
        }

        PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, data);

        convolution.allocate();
        success *= (convolution.verify() == 0);
        convolution.initialize();
        convolution.evaluateResidual();

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();

        const auto& residual = convolution.getResidual();
        for (size_t i = 0; i < residual.size(); ++i)
        {
          if (!isEqual(residual[i], static_cast<ScalarT>(0.0), tol))
          {
            std::cout << "Non-zero residual at index " << i << ": " << residual[i] << "\n";
            success = false;
          }
        }

        for (size_t i = 0; i < data.dimension; ++i)
        {
          success *= nodes.outputs[i].linked();
          success *= (nodes.outputs[i].getVariableIndex() == static_cast<IdxT>(data.dimension + i));
          success *= isEqual(nodes.outputs[i].read(), convolution.y()[data.dimension + i], tol);
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        auto data             = makeTestData();
        auto nodes            = makeSignalNodes();
        nodes.input_values[0] = 0.7;
        nodes.input_values[1] = -0.2;

        PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, data);

        convolution.allocate();
        convolution.initialize();

        convolution.y()[0]  = 0.8;  // u0
        convolution.y()[1]  = -0.1; // u1
        convolution.y()[2]  = 1.2;  // z0
        convolution.y()[3]  = -0.3; // z1
        convolution.y()[4]  = 0.4;  // x0
        convolution.y()[5]  = -0.2; // x1
        convolution.yp()[0] = 0.05;
        convolution.yp()[1] = -0.07;
        convolution.yp()[4] = 0.6;
        convolution.yp()[5] = -0.4;

        convolution.evaluateResidual();

        const std::vector<ScalarT> expected = {
            0.1,    // u0 - U0
            0.1,    // u1 - U1
            0.4709, // z0 - row0(D)*u - row0(E)*u' - sum(c[k,0]*x[k])
            0.0143, // z1 - row1(D)*u - row1(E)*u' - sum(c[k,1]*x[k])
            -0.65,  // -x0' + b0*u + p0*x0
            1.125,  // -x1' + b1*u + p1*x1
        };

        const auto& residual = convolution.getResidual();
        const auto  tol      = static_cast<RealT>(1.0e-12);

        for (size_t i = 0; i < expected.size(); ++i)
        {
          if (!isEqual(residual[i], expected[i], tol))
          {
            std::cout << "Incorrect residual at index " << i << ": "
                      << residual[i] << " != " << expected[i] << "\n";
            success = false;
          }
        }

        success *= isEqual(nodes.outputs[0].read(), static_cast<ScalarT>(1.2), tol);
        success *= isEqual(nodes.outputs[1].read(), static_cast<ScalarT>(-0.3), tol);

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        auto data  = makeTestData();
        auto nodes = makeSignalNodes();

        PhasorDynamics::Convolution::ConvolutionVec<ScalarT, IdxT> convolution(nodes.input_ptrs, nodes.output_ptrs, data);

        convolution.allocate();
        convolution.updateTime(0.0, 10.0);
        convolution.evaluateJacobian();

        auto jacobian = MapFromCOO<RealT, IdxT>(convolution.getJacobian());

        std::vector<DependencyTracking::Variable::DependencyMap> expected(6);
        expected[0][0]  = 1.0;
        expected[0][20] = -1.0;
        expected[1][1]  = 1.0;
        expected[1][21] = -1.0;

        expected[2][0] = -0.3;
        expected[2][1] = -0.1;
        expected[2][2] = 1.0;
        expected[2][4] = -1.5;
        expected[2][5] = -0.2;

        expected[3][0] = 0.25;
        expected[3][1] = -0.7;
        expected[3][3] = 1.0;
        expected[3][4] = 0.4;
        expected[3][5] = -0.8;

        expected[4][0] = 1.0;
        expected[4][1] = 0.5;
        expected[4][4] = -12.0;

        expected[5][0] = -0.25;
        expected[5][1] = 0.75;
        expected[5][5] = -15.0;

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();

        success *= (jacobian.size() == expected.size());
        for (size_t i = 0; i < expected.size() && i < jacobian.size(); ++i)
        {
          if (!isEqual<IdxT, RealT>(jacobian[i], expected[i], tol))
          {
            std::cout << "Incorrect Jacobian row " << i << "\n";
            success = false;
          }
        }

        return success.report(__func__);
      }

    private:
      struct SignalNodes
      {
        std::vector<ScalarT> input_values{0.0, 0.0};
        std::vector<IdxT>    input_indices{20, 21};

        std::vector<PhasorDynamics::SignalNode<ScalarT, IdxT>> inputs{2};
        std::vector<PhasorDynamics::SignalNode<ScalarT, IdxT>> outputs{2};

        std::vector<PhasorDynamics::SignalNode<ScalarT, IdxT>*> input_ptrs;
        std::vector<PhasorDynamics::SignalNode<ScalarT, IdxT>*> output_ptrs;
      };

      static auto makeSignalNodes(bool link_inputs = true) -> SignalNodes
      {
        SignalNodes nodes;
        nodes.input_ptrs  = {&nodes.inputs[0], &nodes.inputs[1]};
        nodes.output_ptrs = {&nodes.outputs[0], &nodes.outputs[1]};

        if (link_inputs)
        {
          for (size_t i = 0; i < nodes.inputs.size(); ++i)
          {
            nodes.inputs[i].set(&nodes.input_values[i], &nodes.input_indices[i]);
          }
        }

        return nodes;
      }

      static auto makeTestData() -> PhasorDynamics::Convolution::ConvolutionVecData<RealT, IdxT>
      {
        PhasorDynamics::Convolution::ConvolutionVecData<RealT, IdxT> data;
        data.dimension = 2;
        data.d         = {0.2, -0.1, 0.05, 0.3};
        data.e         = {0.01, 0.02, -0.03, 0.04};
        data.p         = {-2.0, -5.0};
        data.b         = {1.0, 0.5, -0.25, 0.75};
        data.c         = {1.5, -0.4, 0.2, 0.8};
        data.u0        = {1.0, -0.5};
        data.up0       = {0.2, 0.1};

        return data;
      }
    };

  } // namespace Testing
} // namespace GridKit
