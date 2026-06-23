#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/VectorizedEquations.hpp>
#include <GridKit/ScalarTraits.hpp>
#include <GridKit/Testing/Testing.hpp>

#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/AutomaticDifferentiation/Enzyme/SparseJacobians.hpp>
#include <GridKit/LinearAlgebra/SparseMatrix/COO_Matrix.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>
#endif

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class VectorizedEquationModel
    {
    public:
      using RealT                    = typename GridKit::ScalarTraits<ScalarT>::RealT;
      static constexpr std::size_t N = 3;

      VectorizedEquationModel()
      {
        Y_(0, 0) = 2.0;
        Y_(0, 1) = 3.0;
        Y_(0, 2) = 5.0;
        Y_(1, 0) = 7.0;
        Y_(1, 1) = 11.0;
        Y_(1, 2) = 13.0;
        Y_(2, 0) = 17.0;
        Y_(2, 1) = 19.0;
        Y_(2, 2) = 23.0;
      }

      __attribute__((always_inline)) int evaluateInternalResidual(
          ScalarT* y, [[maybe_unused]] ScalarT* yp, ScalarT* wb, ScalarT* f)
      {
        namespace Eq = GridKit::PhasorDynamics::Equation;

        auto i1 = Eq::block<N>(y, 0);
        auto i2 = Eq::block<N>(y, 1);
        auto v  = Eq::block<N>(wb, 0);

        auto r1 = Eq::block<N>(f, 0);
        auto r2 = Eq::block<N>(f, 1);

        r1 = i1 - Y_ * v;
        r2 = i2 - Y_ * v;

        return 0;
      }

      const GridKit::PhasorDynamics::Equation::Mat<RealT, N, N>& matrix() const
      {
        return Y_;
      }

    private:
      GridKit::PhasorDynamics::Equation::Mat<RealT, N, N> Y_{};
    };

    template <class ScalarT, typename IdxT, std::size_t N, std::size_t M>
    class SignalSumModel
    {
    public:
      __attribute__((always_inline)) int evaluateInternalResidual(
          [[maybe_unused]] ScalarT* y,
          [[maybe_unused]] ScalarT* yp,
          [[maybe_unused]] ScalarT* wb,
          ScalarT*                  ws,
          ScalarT*                  f)
      {
        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT sum = 0.0;
          for (std::size_t m = 0; m < M; ++m)
          {
            sum += ws[m * N + n];
          }
          f[n] = sum;
        }

        return 0;
      }
    };

    template <class ScalarT, typename IdxT>
    class VectorizedEquationsTests
    {
    public:
      using RealT                    = typename GridKit::ScalarTraits<ScalarT>::RealT;
      static constexpr std::size_t N = 3;
      static constexpr std::size_t M = 2;

      TestOutcome blockAssignment()
      {
        TestStatus success = true;

        std::array<ScalarT, 9> values{};
        auto                   dst = GridKit::PhasorDynamics::Equation::block<N>(values.data(), 1);

        GridKit::PhasorDynamics::Equation::Vec<ScalarT, N> rhs{};
        rhs[0] = 1.0;
        rhs[1] = 2.0;
        rhs[2] = 3.0;

        dst = rhs;

        success *= isEqual(values[0], ScalarT{0.0});
        success *= isEqual(values[3], ScalarT{1.0});
        success *= isEqual(values[4], ScalarT{2.0});
        success *= isEqual(values[5], ScalarT{3.0});
        success *= isEqual(values[8], ScalarT{0.0});

        return success.report(__func__);
      }

      TestOutcome sliceAssignment()
      {
        TestStatus success = true;

        std::array<ScalarT, 6> values{};
        auto                   dst = GridKit::PhasorDynamics::Equation::slice<N>(values.data(), 2);

        GridKit::PhasorDynamics::Equation::Vec<ScalarT, N> rhs{};
        rhs[0] = 4.0;
        rhs[1] = 5.0;
        rhs[2] = 6.0;

        dst = rhs;

        success *= isEqual(values[1], ScalarT{0.0});
        success *= isEqual(values[2], ScalarT{4.0});
        success *= isEqual(values[3], ScalarT{5.0});
        success *= isEqual(values[4], ScalarT{6.0});
        success *= isEqual(values[5], ScalarT{0.0});

        return success.report(__func__);
      }

      TestOutcome residualLayout()
      {
        TestStatus success = true;

        VectorizedEquationModel<ScalarT, IdxT> model;
        std::array<ScalarT, 2 * N>             y{1.0, 2.0, 3.0, 10.0, 20.0, 30.0};
        std::array<ScalarT, 2 * N>             yp{};
        std::array<ScalarT, N>                 wb{0.5, -1.0, 2.0};
        std::array<ScalarT, 2 * N>             f{};

        model.evaluateInternalResidual(y.data(), yp.data(), wb.data(), f.data());

        for (std::size_t row = 0; row < N; ++row)
        {
          ScalarT yv = 0.0;
          for (std::size_t col = 0; col < N; ++col)
          {
            yv += model.matrix()(row, col) * wb[col];
          }

          success *= isEqual(f[row], y[row] - yv);
          success *= isEqual(f[N + row], y[N + row] - yv);
        }

        return success.report(__func__);
      }

      TestOutcome signalSumResidualLayout()
      {
        TestStatus success = true;

        SignalSumModel<ScalarT, IdxT, N, M> model;
        std::array<ScalarT, 1>              y{};
        std::array<ScalarT, 1>              yp{};
        std::array<ScalarT, 1>              wb{};
        std::array<ScalarT, M * N>          ws{};
        std::array<ScalarT, N>              f{};

        for (std::size_t m = 0; m < M; ++m)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            ws[m * N + n] = static_cast<ScalarT>((m + 1) * (n + 2));
          }
        }

        model.evaluateInternalResidual(y.data(), yp.data(), wb.data(), ws.data(), f.data());

        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT expected = 0.0;
          for (std::size_t m = 0; m < M; ++m)
          {
            expected += ws[m * N + n];
          }
          success *= isEqual(f[n], expected);
        }

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome enzymeJacobian()
      {
        TestStatus success = true;

        VectorizedEquationModel<ScalarT, IdxT> model;
        std::vector<ScalarT>                   y{1.0, 2.0, 3.0, 10.0, 20.0, 30.0};
        std::vector<ScalarT>                   yp(2 * N, 0.0);
        std::vector<ScalarT>                   wb{0.5, -1.0, 2.0};

        std::vector<IdxT> residual_indices{0, 1, 2, 3, 4, 5};
        std::vector<IdxT> variable_indices{0, 1, 2, 3, 4, 5};
        std::vector<IdxT> wb_indices{6, 7, 8};

        const std::size_t max_nnz =
            std::max(residual_indices.size() * variable_indices.size(),
                     residual_indices.size() * wb_indices.size());

        std::vector<IdxT>  rows(max_nnz);
        std::vector<IdxT>  cols(max_nnz);
        std::vector<RealT> vals(max_nnz);

        GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT> J;

        GridKit::Enzyme::Sparse::DfDy<VectorizedEquationModel<ScalarT, IdxT>,
                                      GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                      ScalarT,
                                      IdxT>::eval(&model,
                                                  residual_indices.size(),
                                                  variable_indices.size(),
                                                  residual_indices.data(),
                                                  variable_indices.data(),
                                                  y.data(),
                                                  yp.data(),
                                                  wb.data(),
                                                  rows.data(),
                                                  cols.data(),
                                                  vals.data(),
                                                  J);

        GridKit::Enzyme::Sparse::DfDwb<VectorizedEquationModel<ScalarT, IdxT>,
                                       GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                       ScalarT,
                                       IdxT>::eval(&model,
                                                   residual_indices.size(),
                                                   wb_indices.size(),
                                                   residual_indices.data(),
                                                   wb_indices.data(),
                                                   y.data(),
                                                   yp.data(),
                                                   wb.data(),
                                                   rows.data(),
                                                   cols.data(),
                                                   vals.data(),
                                                   J);

        auto actual   = GridKit::Testing::MapFromCOO(J);
        auto expected = expectedJacobian(model.matrix());

        for (std::size_t row = 0; row < expected.size(); ++row)
        {
          success *= isEqual(actual[row], expected[row]);
        }

        return success.report(__func__);
      }

      TestOutcome signalSumEnzymeJacobian()
      {
        TestStatus success = true;

        SignalSumModel<ScalarT, IdxT, N, M> model;
        std::vector<ScalarT>                y(1, 0.0);
        std::vector<ScalarT>                yp(1, 0.0);
        std::vector<ScalarT>                wb(1, 0.0);
        std::vector<ScalarT>                ws(M * N);

        for (std::size_t m = 0; m < M; ++m)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            ws[m * N + n] = static_cast<ScalarT>((m + 1) * (n + 2));
          }
        }

        std::vector<IdxT> residual_indices(N);
        std::vector<IdxT> ws_indices(M * N);

        for (std::size_t n = 0; n < N; ++n)
        {
          residual_indices[n] = static_cast<IdxT>(n);
        }

        for (std::size_t m = 0; m < M; ++m)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            ws_indices[m * N + n] = static_cast<IdxT>(N + m * N + n);
          }
        }

        const std::size_t max_nnz = residual_indices.size() * ws_indices.size();

        std::vector<IdxT>  rows(max_nnz);
        std::vector<IdxT>  cols(max_nnz);
        std::vector<RealT> vals(max_nnz);

        GridKit::LinearAlgebra::COO_Matrix<RealT, IdxT> J;

        GridKit::Enzyme::Sparse::DfDws<SignalSumModel<ScalarT, IdxT, N, M>,
                                       GridKit::Enzyme::Sparse::MemberFunctions::InternalResidualWithSignal,
                                       ScalarT,
                                       IdxT>::eval(&model,
                                                   residual_indices.size(),
                                                   ws_indices.size(),
                                                   residual_indices.data(),
                                                   ws_indices.data(),
                                                   y.data(),
                                                   yp.data(),
                                                   wb.data(),
                                                   ws.data(),
                                                   rows.data(),
                                                   cols.data(),
                                                   vals.data(),
                                                   J);

        auto actual   = GridKit::Testing::MapFromCOO(J);
        auto expected = expectedSignalSumJacobian();

        for (std::size_t row = 0; row < expected.size(); ++row)
        {
          success *= isEqual(actual[row], expected[row]);
        }

        return success.report(__func__);
      }
#endif

    private:
      std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expectedJacobian(
          const GridKit::PhasorDynamics::Equation::Mat<RealT, N, N>& Y)
      {
        std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expected(2 * N);

        for (std::size_t row = 0; row < N; ++row)
        {
          expected[row][row]         = 1.0;
          expected[N + row][N + row] = 1.0;

          for (std::size_t col = 0; col < N; ++col)
          {
            expected[row][2 * N + col]     = -Y(row, col);
            expected[N + row][2 * N + col] = -Y(row, col);
          }
        }

        return expected;
      }

      std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expectedSignalSumJacobian()
      {
        std::vector<GridKit::DependencyTracking::Variable::DependencyMap> expected(N);

        for (std::size_t n = 0; n < N; ++n)
        {
          for (std::size_t m = 0; m < M; ++m)
          {
            expected[n][static_cast<IdxT>(N + m * N + n)] = 1.0;
          }
        }

        return expected;
      }
    };
  } // namespace Testing
} // namespace GridKit
