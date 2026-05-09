#pragma once

#include <iostream>
#include <limits>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVF.hpp>
#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVFData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class ConvolutionVFTests
    {
    public:
      using RealT = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;

      ConvolutionVFTests()  = default;
      ~ConvolutionVFTests() = default;

      TestOutcome constructor()
      {
        TestStatus success = true;

        PhasorDynamics::SignalNode<ScalarT, IdxT> input_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> output_node;

        auto* convolution =
            new PhasorDynamics::Convolution::ConvolutionVF<ScalarT, IdxT>(&input_node, &output_node, makeTestData());

        success *= (convolution != nullptr);

        delete convolution;

        return success.report(__func__);
      }

      TestOutcome zeroInitialResidual()
      {
        TestStatus success = true;

        auto data = makeTestData();

        ScalarT input_value = data.u0;
        IdxT    input_index = 17;

        PhasorDynamics::SignalNode<ScalarT, IdxT> input_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> output_node;
        input_node.set(&input_value, &input_index);

        PhasorDynamics::Convolution::ConvolutionVF<ScalarT, IdxT> convolution(&input_node, &output_node, data);

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

        success *= output_node.linked();
        success *= (output_node.getVariableIndex() == 1);
        success *= isEqual(output_node.read(), convolution.y()[1], tol);

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        auto data = makeTestData();

        ScalarT input_value = 0.8;
        IdxT    input_index = 17;

        PhasorDynamics::SignalNode<ScalarT, IdxT> input_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> output_node;
        input_node.set(&input_value, &input_index);

        PhasorDynamics::Convolution::ConvolutionVF<ScalarT, IdxT> convolution(&input_node, &output_node, data);

        convolution.allocate();
        convolution.initialize();

        convolution.y()[0]  = 0.9;
        convolution.y()[1]  = 1.1;
        convolution.y()[2]  = 0.2;
        convolution.y()[3]  = -0.1;
        convolution.yp()[0] = 0.3;
        convolution.yp()[2] = 0.4;
        convolution.yp()[3] = 0.6;

        convolution.evaluateResidual();

        const std::vector<ScalarT> expected = {
            0.1,   // u - U
            0.565, // z - d*u - e*u' - sum(r[n]*x[n])
            0.1,   // -x0' + u + p0*x0
            1.0,   // -x1' + u + p1*x1
        };

        const auto& residual = convolution.getResidual();
        const auto  tol      = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();

        for (size_t i = 0; i < expected.size(); ++i)
        {
          if (!isEqual(residual[i], expected[i], tol))
          {
            std::cout << "Incorrect residual at index " << i << ": "
                      << residual[i] << " != " << expected[i] << "\n";
            success = false;
          }
        }

        success *= isEqual(output_node.read(), static_cast<ScalarT>(1.1), tol);

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        auto data = makeTestData();

        ScalarT input_value = 0.0;
        IdxT    input_index = 42;

        PhasorDynamics::SignalNode<ScalarT, IdxT> input_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> output_node;
        input_node.set(&input_value, &input_index);

        PhasorDynamics::Convolution::ConvolutionVF<ScalarT, IdxT> convolution(&input_node, &output_node, data);

        convolution.allocate();
        convolution.updateTime(0.0, 10.0);
        convolution.evaluateJacobian();

        auto jacobian = MapFromCOO<RealT, IdxT>(convolution.getJacobian());

        std::vector<DependencyTracking::Variable::DependencyMap> expected(4);
        expected[0][0]           = 1.0;
        expected[0][input_index] = -1.0;
        expected[1][0]           = -0.7;
        expected[1][1]           = 1.0;
        expected[1][2]           = -1.5;
        expected[1][3]           = 0.4;
        expected[2][0]           = 1.0;
        expected[2][2]           = -12.0;
        expected[3][0]           = 1.0;
        expected[3][3]           = -17.0;

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
      static auto makeTestData() -> PhasorDynamics::Convolution::ConvolutionVFData<RealT, IdxT>
      {
        PhasorDynamics::Convolution::ConvolutionVFData<RealT, IdxT> data;
        data.d   = 0.2;
        data.e   = 0.05;
        data.u0  = 1.0;
        data.up0 = 0.3;
        data.p   = {-2.0, -7.0};
        data.r   = {1.5, -0.4};

        return data;
      }
    };

  } // namespace Testing
} // namespace GridKit
