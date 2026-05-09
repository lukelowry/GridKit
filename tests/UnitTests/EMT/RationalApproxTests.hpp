#pragma once

#include <complex>
#include <iostream>
#include <limits>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Model/EMT/RationalApprox/RationalApprox.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class RationalApproxTests
    {
    public:
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using DataT = EMT::RationalApproxData<RealT, IdxT>;

      TestOutcome constructor()
      {
        TestStatus success = true;

        auto                               data = makeComplexData();
        EMT::RationalApprox<ScalarT, IdxT> approximation(data);

        success *= (approximation.dimension() == data.dimension);
        success *= (approximation.realPoleCount() == data.p.size());
        success *= (approximation.complexPairCount() == data.complex_p_real.size());
        success *= (approximation.modeCount() == data.p.size() + data.complex_p_real.size());
        success *= (approximation.stateCount() == data.p.size() + static_cast<size_t>(2) * data.complex_p_real.size());
        success *= approximation.hasDerivativeFeedthrough();

        auto direct_only = makeRealData();
        direct_only.p.clear();
        direct_only.b.clear();
        direct_only.c.clear();
        direct_only.e.assign(direct_only.dimension * direct_only.dimension, 0.0);

        EMT::RationalApprox<ScalarT, IdxT> direct(direct_only);
        success *= (direct.verify() == 0);
        success *= (direct.stateCount() == 0);
        success *= !direct.hasDerivativeFeedthrough();

        return success.report(__func__);
      }

      TestOutcome verifyFailures()
      {
        TestStatus success = true;

        {
          EMT::RationalApprox<ScalarT, IdxT> approximation(makeComplexData());
          success *= (approximation.verify() == 0);
        }

        {
          auto data      = makeComplexData();
          data.dimension = 0;
          EMT::RationalApprox<ScalarT, IdxT> approximation(data);
          success *= (approximation.verify() > 0);
        }

        {
          auto data = makeComplexData();
          data.d.pop_back();
          EMT::RationalApprox<ScalarT, IdxT> approximation(data);
          success *= (approximation.verify() > 0);
        }

        {
          auto data = makeComplexData();
          data.b.pop_back();
          EMT::RationalApprox<ScalarT, IdxT> approximation(data);
          success *= (approximation.verify() > 0);
        }

        {
          auto data = makeComplexData();
          data.complex_c_imag.pop_back();
          EMT::RationalApprox<ScalarT, IdxT> approximation(data);
          success *= (approximation.verify() > 0);
        }

        {
          auto data              = makeComplexData();
          data.complex_p_imag[0] = 0.0;
          EMT::RationalApprox<ScalarT, IdxT> approximation(data);
          success *= (approximation.verify() > 0);
        }

        return success.report(__func__);
      }

      TestOutcome initialization()
      {
        TestStatus success = true;

        auto                               data = makeComplexData();
        EMT::RationalApprox<ScalarT, IdxT> approximation(data);

        std::vector<ScalarT> state(approximation.stateCount(), 0.0);
        std::vector<ScalarT> state_derivative(approximation.stateCount(), 0.0);
        std::vector<ScalarT> residual(approximation.stateCount(), 0.0);

        const std::vector<ScalarT> u0{1.0, -0.25};
        const std::vector<ScalarT> up0{0.2, -0.1};

        approximation.initialize(u0.data(), up0.data(), state.data(), state_derivative.data());
        approximation.evaluateStateResidual(u0.data(), state.data(), state_derivative.data(), residual.data());

        const auto tol = static_cast<RealT>(1000.0) * std::numeric_limits<RealT>::epsilon();
        for (size_t i = 0; i < residual.size(); ++i)
        {
          if (!isEqual(residual[i], static_cast<ScalarT>(0.0), tol))
          {
            std::cout << "Non-zero RationalApprox initialized residual at index " << i << ": " << residual[i] << "\n";
            success = false;
          }
        }

        return success.report(__func__);
      }

      TestOutcome outputAndResidual()
      {
        TestStatus success = true;

        auto                               data = makeComplexData();
        EMT::RationalApprox<ScalarT, IdxT> approximation(data);

        const std::vector<ScalarT> u{0.5, -0.25};
        const std::vector<ScalarT> up{0.08, -0.06};
        const std::vector<ScalarT> state{0.2, -0.4, 0.15};
        const std::vector<ScalarT> state_derivative{-0.3, 0.11, -0.22};
        std::vector<ScalarT>       output(2, 0.0);
        std::vector<ScalarT>       residual(3, 0.0);

        approximation.evaluateOutput(u.data(), up.data(), state.data(), output.data());
        approximation.evaluateStateResidual(u.data(), state.data(), state_derivative.data(), residual.data());

        const std::vector<ScalarT> expected_output{-0.3755, -0.04};
        const std::vector<ScalarT> expected_residual{-0.3, 0.665, -1.2675};
        const auto                 tol = static_cast<RealT>(1.0e-12);

        for (size_t i = 0; i < output.size(); ++i)
        {
          success *= isEqual(output[i], expected_output[i], tol);
        }
        for (size_t i = 0; i < residual.size(); ++i)
        {
          success *= isEqual(residual[i], expected_residual[i], tol);
        }

        return success.report(__func__);
      }

      TestOutcome jacobianEntries()
      {
        TestStatus success = true;

        auto                               data = makeComplexData();
        EMT::RationalApprox<ScalarT, IdxT> approximation(data);

        std::vector<DependencyTracking::Variable::DependencyMap> expected_output(2);
        expected_output[0][0] = -0.2;
        expected_output[0][1] = 0.25;
        expected_output[0][2] = -0.7;
        expected_output[0][3] = -1.6;
        expected_output[0][4] = -0.4;
        expected_output[1][0] = -0.32;
        expected_output[1][1] = -0.52;
        expected_output[1][2] = 0.4;
        expected_output[1][3] = 0.6;
        expected_output[1][4] = 1.2;

        std::vector<DependencyTracking::Variable::DependencyMap> output(2);
        auto                                                     add_output = [&](IdxT row, IdxT col, RealT value)
        {
          output[static_cast<size_t>(row)][col] = value;
        };
        approximation.addOutputJacobianEntries(0, 0, 2, 10.0, -1.0, add_output);

        std::vector<DependencyTracking::Variable::DependencyMap> expected_state(3);
        expected_state[0][0] = 0.3;
        expected_state[0][1] = -0.2;
        expected_state[0][2] = -14.0;
        expected_state[1][0] = 1.0;
        expected_state[1][1] = -0.5;
        expected_state[1][3] = -11.5;
        expected_state[1][4] = -3.0;
        expected_state[2][0] = 0.25;
        expected_state[2][1] = 0.75;
        expected_state[2][3] = 3.0;
        expected_state[2][4] = -11.5;

        std::vector<DependencyTracking::Variable::DependencyMap> state(3);
        auto                                                     add_state = [&](IdxT row, IdxT col, RealT value)
        {
          state[static_cast<size_t>(row)][col] = value;
        };
        approximation.addStateJacobianEntries(0, 0, 2, 10.0, add_state);

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();
        for (size_t i = 0; i < output.size(); ++i)
        {
          success *= isEqual<IdxT, RealT>(output[i], expected_output[i], tol);
        }
        for (size_t i = 0; i < state.size(); ++i)
        {
          success *= isEqual<IdxT, RealT>(state[i], expected_state[i], tol);
        }

        return success.report(__func__);
      }

      TestOutcome frequencyResponse()
      {
        TestStatus success = true;

        using ComplexT = std::complex<RealT>;

        auto                               data = makeComplexData();
        EMT::RationalApprox<ScalarT, IdxT> approximation(data);

        const RealT                 omega = 2.3;
        const ComplexT              s{0.0, omega};
        const std::vector<ComplexT> input{{1.0, -0.4}, {-0.25, 0.7}};

        std::vector<ComplexT> output_direct(data.dimension, ComplexT{0.0, 0.0});
        for (size_t i = 0; i < data.dimension; ++i)
        {
          for (size_t j = 0; j < data.dimension; ++j)
          {
            output_direct[i] += data.d[i * data.dimension + j] * input[j];
            output_direct[i] += data.e[i * data.dimension + j] * s * input[j];
          }
        }

        std::vector<ComplexT> state_complex(approximation.stateCount(), ComplexT{0.0, 0.0});

        for (size_t k = 0; k < data.p.size(); ++k)
        {
          ComplexT modal_input{0.0, 0.0};
          for (size_t i = 0; i < data.dimension; ++i)
          {
            modal_input += data.b[k * data.dimension + i] * input[i];
          }
          state_complex[k] = modal_input / (s - data.p[k]);
          for (size_t i = 0; i < data.dimension; ++i)
          {
            output_direct[i] += data.c[k * data.dimension + i] * state_complex[k];
          }
        }

        const auto complex_state_offset = data.p.size();
        for (size_t pair = 0; pair < data.complex_p_real.size(); ++pair)
        {
          ComplexT input_real{0.0, 0.0};
          ComplexT input_imag{0.0, 0.0};
          ComplexT input_plus{0.0, 0.0};
          ComplexT input_minus{0.0, 0.0};
          for (size_t i = 0; i < data.dimension; ++i)
          {
            const auto     index = pair * data.dimension + i;
            const ComplexT coupling_plus{data.complex_b_real[index], data.complex_b_imag[index]};
            const ComplexT coupling_minus{data.complex_b_real[index], -data.complex_b_imag[index]};

            input_real  += data.complex_b_real[index] * input[i];
            input_imag  += data.complex_b_imag[index] * input[i];
            input_plus  += coupling_plus * input[i];
            input_minus += coupling_minus * input[i];
          }

          const auto     pole_real = data.complex_p_real[pair];
          const auto     pole_imag = data.complex_p_imag[pair];
          const ComplexT pole_plus{pole_real, pole_imag};
          const ComplexT pole_minus{pole_real, -pole_imag};
          const ComplexT block_denom = (s - pole_real) * (s - pole_real) + pole_imag * pole_imag;

          state_complex[complex_state_offset + static_cast<size_t>(2) * pair] =
              ((s - pole_real) * input_real - pole_imag * input_imag) / block_denom;
          state_complex[complex_state_offset + static_cast<size_t>(2) * pair + 1] =
              (pole_imag * input_real + (s - pole_real) * input_imag) / block_denom;

          for (size_t i = 0; i < data.dimension; ++i)
          {
            const auto     index = pair * data.dimension + i;
            const ComplexT residue_plus{data.complex_c_real[index], data.complex_c_imag[index]};
            const ComplexT residue_minus{data.complex_c_real[index], -data.complex_c_imag[index]};
            output_direct[i] += residue_plus * input_plus / (s - pole_plus);
            output_direct[i] += residue_minus * input_minus / (s - pole_minus);
          }
        }

        std::vector<ScalarT> u(data.dimension);
        std::vector<ScalarT> up(data.dimension);
        std::vector<ScalarT> state(approximation.stateCount());
        std::vector<ScalarT> state_derivative(approximation.stateCount());
        std::vector<ScalarT> output(data.dimension);
        std::vector<ScalarT> residual(approximation.stateCount());

        for (size_t i = 0; i < data.dimension; ++i)
        {
          u[i]  = input[i].real();
          up[i] = (s * input[i]).real();
        }
        for (size_t i = 0; i < state.size(); ++i)
        {
          state[i]            = state_complex[i].real();
          state_derivative[i] = (s * state_complex[i]).real();
        }

        approximation.evaluateStateResidual(u.data(), state.data(), state_derivative.data(), residual.data());
        approximation.evaluateOutput(u.data(), up.data(), state.data(), output.data());

        const auto tol = static_cast<RealT>(1.0e-10);
        for (auto value : residual)
        {
          success *= isEqual(value, static_cast<ScalarT>(0.0), tol);
        }
        for (size_t i = 0; i < output.size(); ++i)
        {
          success *= isEqual(output[i], static_cast<ScalarT>(output_direct[i].real()), tol);
        }

        return success.report(__func__);
      }

    private:
      static auto makeRealData() -> DataT
      {
        DataT data;
        data.dimension = 2;
        data.d         = {0.2, -0.1, 0.05, 0.3};
        data.e         = {0.01, 0.02, -0.03, 0.04};
        data.p         = {-2.0, -5.0};
        data.b         = {1.0, 0.5, -0.25, 0.75};
        data.c         = {1.5, -0.4, 0.2, 0.8};
        return data;
      }

      static auto makeComplexData() -> DataT
      {
        DataT data;
        data.dimension = 2;
        data.d         = {0.1, -0.05, 0.02, 0.12};
        data.e         = {0.01, -0.02, 0.03, 0.04};
        data.p         = {-4.0};
        data.b         = {0.3, -0.2};
        data.c         = {0.7, -0.4};

        data.complex_p_real = {-1.5};
        data.complex_p_imag = {3.0};
        data.complex_b_real = {1.0, -0.5};
        data.complex_b_imag = {0.25, 0.75};
        data.complex_c_real = {0.8, -0.3};
        data.complex_c_imag = {-0.2, 0.6};

        return data;
      }
    };

  } // namespace Testing
} // namespace GridKit
