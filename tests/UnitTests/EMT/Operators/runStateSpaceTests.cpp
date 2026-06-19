#include <cmath>
#include <complex>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpace.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace
{
  using scalar_type    = double;
  using index_type     = size_t;
  using StateSpace     = GridKit::EMT::Operators::Rational::StateSpace<scalar_type, index_type>;
  using StateSpaceData = GridKit::EMT::Operators::Rational::StateSpaceData<scalar_type, index_type>;
  using Signal         = StateSpace::SignalT;
  using Storage        = StateSpace::SignalStorageT;
  using Complex        = std::complex<scalar_type>;

  bool close(scalar_type value, scalar_type ref, scalar_type tol = 1.0e-10)
  {
    return std::abs(value - ref) <= tol * (1.0 + std::abs(ref));
  }

  StateSpaceData makeData()
  {
    StateSpaceData data;
    data.N     = 2;
    data.K     = 2;
    data.Q     = 2;
    data.D     = {1.0, -2.0, 0.5, 3.0};
    data.E     = {0.1, 0.2, -0.3, 0.4};
    data.poles = {Complex{-2.0, 3.0}, Complex{-2.0, -3.0}};
    data.C     = {Complex{1.0, 0.5},
                  Complex{1.0, -0.5},
                  Complex{-0.25, 0.75},
                  Complex{-0.25, -0.75}};
    data.B     = {Complex{2.0, -1.0},
                  Complex{-0.5, 0.25},
                  Complex{2.0, 1.0},
                  Complex{-0.5, -0.25}};
    return data;
  }

  StateSpaceData makeRealPoleData()
  {
    StateSpaceData data;
    data.N     = 2;
    data.K     = 2;
    data.Q     = 1;
    data.D     = {1.0, -2.0, 0.5, 3.0};
    data.E     = {0.1, 0.2, -0.3, 0.4};
    data.poles = {Complex{-2.0, 0.0}};
    data.C     = {Complex{1.0, 0.0}, Complex{-0.25, 0.0}};
    data.B     = {Complex{2.0, 0.0}, Complex{-0.5, 0.0}};
    return data;
  }

  Signal vectorInput(index_type offset = 0)
  {
    return {offset, 2, 1};
  }

  scalar_type cooValue(StateSpace::MatrixT& J, index_type row, index_type col)
  {
    auto [rows, cols, vals] = J.getEntryCopies();
    scalar_type value       = 0.0;
    for (size_t i = 0; i < vals.size(); ++i)
    {
      if (rows[i] == row && cols[i] == col)
      {
        value += vals[i];
      }
    }
    return value;
  }

  GridKit::Testing::TestOutcome state_space_rejects_invalid_contracts()
  {
    using GridKit::Testing::TestStatus;

    TestStatus success = true;

    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.N    = 0;
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.K    = 0;
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.Q    = 0;
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.D.pop_back();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.E.pop_back();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.poles.pop_back();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.C.pop_back();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.B.pop_back();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.D[0] = std::numeric_limits<scalar_type>::infinity();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data     = makeData();
          data.poles[0] = Complex{0.0, 0.0};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data     = makeData();
          data.poles[1] = Complex{-1.0, -3.0};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.C[1] = Complex{1.0, -0.25};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.B[2] = Complex{2.0, 0.5};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeRealPoleData();
          data.C[0] = Complex{1.0, 0.1};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeRealPoleData();
          data.B[0] = Complex{2.0, -0.1};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.E[0] = std::numeric_limits<scalar_type>::quiet_NaN();
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.C[0] = Complex{1.0, std::numeric_limits<scalar_type>::infinity()};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.B[0] = Complex{std::numeric_limits<scalar_type>::quiet_NaN(), 0.0};
          StateSpace model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          StateSpace model(makeData(), Signal{0, 1, 1});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          StateSpace model(makeData(), Signal{0, 2, 2});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          StateSpace model(makeData(), Signal{0, 2, 1, Storage::Derivative});
        });

    StateSpace model(makeRealPoleData(), vectorInput());
    success *= (model.size() == 4);

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome state_space_initializes_from_matrix_equations()
  {
    using GridKit::Testing::TestStatus;

    const auto       data = makeData();
    const index_type base = 3;
    StateSpace       model(data, vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0]  = 1.5;
    y[1]  = -0.75;
    yp[0] = 0.2;
    yp[1] = -0.1;

    model.bind(y.data(), yp.data(), f.data(), index.data(), base);
    model.initialize();

    std::vector<scalar_type> xr(data.Q, 0.0);
    std::vector<scalar_type> xi(data.Q, 0.0);
    std::vector<scalar_type> xrd(data.Q, 0.0);
    std::vector<scalar_type> xid(data.Q, 0.0);
    for (index_type q = 0; q < data.Q; ++q)
    {
      Complex Bu{0.0, 0.0};
      Complex Bud{0.0, 0.0};
      for (index_type k = 0; k < data.K; ++k)
      {
        const auto Bqk  = data.B[q * data.K + k];
        Bu             += Bqk * y[k];
        Bud            += Bqk * yp[k];
      }

      const auto p  = data.poles[q];
      const auto x0 = -Bu / p - Bud / (p * p);
      xr[q]         = x0.real();
      xi[q]         = x0.imag();
      xrd[q]        = p.real() * xr[q] - p.imag() * xi[q] + Bu.real();
      xid[q]        = p.imag() * xr[q] + p.real() * xi[q] + Bu.imag();
    }

    std::vector<scalar_type> out(data.N, 0.0);
    std::vector<scalar_type> yd(data.N, 0.0);
    for (index_type i = 0; i < data.N; ++i)
    {
      for (index_type k = 0; k < data.K; ++k)
      {
        out[i] += data.D[i * data.K + k] * y[k] + data.E[i * data.K + k] * yp[k];
        yd[i]  += data.D[i * data.K + k] * yp[k];
      }
      for (index_type q = 0; q < data.Q; ++q)
      {
        const auto Ciq  = data.C[i * data.Q + q];
        out[i]         += Ciq.real() * xr[q] - Ciq.imag() * xi[q];
        yd[i]          += Ciq.real() * xrd[q] - Ciq.imag() * xid[q];
      }
    }

    const index_type xr_offset  = 0;
    const index_type xi_offset  = data.Q;
    const index_type out_offset = 2 * data.Q;

    const auto out_signal = model.out();
    auto       inputs     = model.inputs();

    TestStatus success  = true;
    success            *= (model.size() == 2 * data.Q + data.N);
    success            *= (inputs.size() == 2);
    success            *= (inputs[0].offset == 0);
    success            *= (inputs[0].rows == data.K);
    success            *= (inputs[0].storage == Storage::State);
    success            *= (inputs[1].offset == 0);
    success            *= (inputs[1].rows == data.K);
    success            *= (inputs[1].storage == Storage::Derivative);
    success            *= (out_signal.offset == base + out_offset);
    success            *= (out_signal.rows == data.N);
    success            *= (out_signal.cols == 1);
    success            *= (out_signal.storage == Storage::State);

    for (index_type q = 0; q < data.Q; ++q)
    {
      success *= close(y[base + xr_offset + q], xr[q]);
      success *= close(y[base + xi_offset + q], xi[q]);
      success *= close(yp[base + xr_offset + q], xrd[q]);
      success *= close(yp[base + xi_offset + q], xid[q]);
    }
    for (index_type i = 0; i < data.N; ++i)
    {
      success *= close(y[base + out_offset + i], out[i]);
      success *= close(yp[base + out_offset + i], yd[i]);
    }

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome state_space_residual_matches_matrix_equations()
  {
    using GridKit::Testing::TestStatus;

    const auto       data       = makeData();
    const index_type base       = 2;
    const index_type xr_offset  = 0;
    const index_type xi_offset  = data.Q;
    const index_type out_offset = 2 * data.Q;
    StateSpace       model(data, vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0]  = -1.0;
    y[1]  = 2.0;
    yp[0] = 0.25;
    yp[1] = -0.5;

    y[base + xr_offset + 0]  = 0.1;
    y[base + xr_offset + 1]  = -0.2;
    y[base + xi_offset + 0]  = 0.3;
    y[base + xi_offset + 1]  = -0.4;
    y[base + out_offset + 0] = 1.25;
    y[base + out_offset + 1] = -0.75;

    yp[base + xr_offset + 0] = -0.6;
    yp[base + xr_offset + 1] = 0.7;
    yp[base + xi_offset + 0] = -0.8;
    yp[base + xi_offset + 1] = 0.9;

    model.bind(y.data(), yp.data(), f.data(), index.data(), base);
    model.evaluateResidual();

    TestStatus success = true;
    for (index_type q = 0; q < data.Q; ++q)
    {
      scalar_type BrU = 0.0;
      scalar_type BiU = 0.0;
      for (index_type k = 0; k < data.K; ++k)
      {
        BrU += data.B[q * data.K + k].real() * y[k];
        BiU += data.B[q * data.K + k].imag() * y[k];
      }

      const auto xr  = y[base + xr_offset + q];
      const auto xi  = y[base + xi_offset + q];
      const auto xrd = yp[base + xr_offset + q];
      const auto xid = yp[base + xi_offset + q];
      const auto a   = data.poles[q].real();
      const auto w   = data.poles[q].imag();

      success *= close(f[base + xr_offset + q], -xrd + a * xr - w * xi + BrU);
      success *= close(f[base + xi_offset + q], -xid + w * xr + a * xi + BiU);
    }

    for (index_type i = 0; i < data.N; ++i)
    {
      scalar_type value = -y[base + out_offset + i];
      for (index_type k = 0; k < data.K; ++k)
      {
        value += data.D[i * data.K + k] * y[k] + data.E[i * data.K + k] * yp[k];
      }
      for (index_type q = 0; q < data.Q; ++q)
      {
        value += data.C[i * data.Q + q].real() * y[base + xr_offset + q]
                 - data.C[i * data.Q + q].imag() * y[base + xi_offset + q];
      }
      success *= close(f[base + out_offset + i], value);
    }

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome state_space_sparse_jet_jacobian_matches_equations()
  {
    using GridKit::Testing::TestStatus;

    const auto        data       = makeData();
    const index_type  base       = 2;
    const index_type  xr_offset  = 0;
    const index_type  xi_offset  = data.Q;
    const index_type  out_offset = 2 * data.Q;
    const scalar_type alpha      = 5.0;
    StateSpace        model(data, vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    model.bind(y.data(), yp.data(), f.data(), index.data(), base);
    model.setCoordinate(0.0, alpha);
    model.evaluateJacobian();
    auto& J = model.jacobian();

    const index_type row_xr0  = base + xr_offset;
    const index_type row_xi0  = base + xi_offset;
    const index_type row_out0 = base + out_offset;

    TestStatus success  = true;
    success            *= (J.nnz() > 0);
    success            *= close(cooValue(J, row_xr0, base + xr_offset), data.poles[0].real() - alpha);
    success            *= close(cooValue(J, row_xr0, base + xi_offset), -data.poles[0].imag());
    success            *= close(cooValue(J, row_xr0, 0), data.B[0].real());
    success            *= close(cooValue(J, row_xr0, 1), data.B[1].real());

    success *= close(cooValue(J, row_xi0, base + xr_offset), data.poles[0].imag());
    success *= close(cooValue(J, row_xi0, base + xi_offset), data.poles[0].real() - alpha);
    success *= close(cooValue(J, row_xi0, 0), data.B[0].imag());
    success *= close(cooValue(J, row_xi0, 1), data.B[1].imag());

    success *= close(cooValue(J, row_out0, base + out_offset), -1.0);
    success *= close(cooValue(J, row_out0, 0), data.D[0] + alpha * data.E[0]);
    success *= close(cooValue(J, row_out0, 1), data.D[1] + alpha * data.E[1]);
    success *= close(cooValue(J, row_out0, base + xr_offset), data.C[0].real());
    success *= close(cooValue(J, row_out0, base + xi_offset), -data.C[0].imag());
    success *= close(cooValue(J, row_out0, base + xr_offset + 1), data.C[1].real());
    success *= close(cooValue(J, row_out0, base + xi_offset + 1), -data.C[1].imag());

    std::vector<bool> tag(base + model.size(), false);
    model.tagDifferentiable(tag);
    success *= !tag[0];
    success *= !tag[1];
    for (index_type q = 0; q < data.Q; ++q)
    {
      success *= tag[base + xr_offset + q];
      success *= tag[base + xi_offset + q];
    }
    for (index_type i = 0; i < data.N; ++i)
    {
      success *= !tag[base + out_offset + i];
    }

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += state_space_rejects_invalid_contracts();
  result += state_space_initializes_from_matrix_equations();
  result += state_space_residual_matches_matrix_equations();
  result += state_space_sparse_jet_jacobian_matches_equations();
  return result.summary();
}
