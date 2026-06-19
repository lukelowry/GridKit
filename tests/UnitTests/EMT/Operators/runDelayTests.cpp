#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/EMT/Operators/Shift/Delay/Delay.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace
{
  using scalar_type = double;
  using index_type  = size_t;
  using Delay       = GridKit::EMT::Operators::Shift::Delay<scalar_type, index_type>;
  using DelayData   = GridKit::EMT::Operators::Shift::DelayData<scalar_type, index_type>;
  using Signal      = Delay::SignalT;
  using Storage     = Delay::SignalStorageT;

  bool close(scalar_type value, scalar_type ref, scalar_type tol = 1.0e-10)
  {
    return std::abs(value - ref) <= tol * (1.0 + std::abs(ref));
  }

  DelayData makeData(scalar_type delay = 4.0, scalar_type dt_min = 1.0)
  {
    DelayData data;
    data.delay  = delay;
    data.dt_min = dt_min;
    return data;
  }

  Signal scalarInput(index_type offset = 0)
  {
    return {offset, 1, 1};
  }

  scalar_type cooValue(Delay::MatrixT& J, index_type row, index_type col)
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

  GridKit::Testing::TestOutcome delay_rejects_invalid_contracts()
  {
    using GridKit::Testing::TestStatus;

    TestStatus success = true;

    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(0.0, 1.0), scalarInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(std::numeric_limits<scalar_type>::infinity(), 1.0), scalarInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(1.0, 0.0), scalarInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(1.0, std::numeric_limits<scalar_type>::quiet_NaN()), scalarInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(std::numeric_limits<scalar_type>::max(),
                               std::numeric_limits<scalar_type>::min()),
                      scalarInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(), Signal{0, 2, 1});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(), Signal{0, 1, 2});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Delay delay(makeData(), Signal{0, 1, 1, Storage::Derivative});
        });

    Delay one_lag(makeData(0.5, 1.0), scalarInput());
    success *= (one_lag.size() == 1);

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome delay_initializes_from_scalar_input()
  {
    using GridKit::Testing::TestStatus;

    const index_type base = 1;
    Delay            delay(makeData(), scalarInput());

    std::vector<scalar_type> y(base + delay.size(), 0.0);
    std::vector<scalar_type> yp(base + delay.size(), -1.0);
    std::vector<scalar_type> f(base + delay.size(), 0.0);
    std::vector<index_type>  index(base + delay.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0] = 3.5;
    delay.bind(y.data(), yp.data(), f.data(), index.data(), base);
    delay.initialize();

    const auto out = delay.out();

    TestStatus success  = true;
    success            *= (delay.size() == 4);
    success            *= (delay.inputs().size() == 1);
    success            *= (delay.inputs()[0].offset == 0);
    success            *= (out.offset == base + delay.size() - 1);
    success            *= (out.rows == 1);
    success            *= (out.cols == 1);
    for (index_type i = 0; i < delay.size(); ++i)
    {
      success *= close(y[base + i], y[0]);
      success *= close(yp[base + i], 0.0);
    }

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome delay_residual_matches_lag_chain()
  {
    using GridKit::Testing::TestStatus;

    const index_type  base      = 1;
    const scalar_type tau       = 4.0;
    const scalar_type lag_count = 4.0;
    Delay             delay(makeData(tau, 1.0), scalarInput());

    std::vector<scalar_type> y(base + delay.size(), 0.0);
    std::vector<scalar_type> yp(base + delay.size(), 0.0);
    std::vector<scalar_type> f(base + delay.size(), 0.0);
    std::vector<index_type>  index(base + delay.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0]        = 10.0;
    y[base + 0] = 7.0;
    y[base + 1] = 6.0;
    y[base + 2] = 5.0;
    y[base + 3] = 3.0;

    yp[base + 0] = 0.25;
    yp[base + 1] = -0.5;
    yp[base + 2] = 1.0;
    yp[base + 3] = -2.0;

    delay.bind(y.data(), yp.data(), f.data(), index.data(), base);
    delay.evaluateResidual();

    TestStatus success  = true;
    success            *= close(f[base + 0], -tau * yp[base + 0] + lag_count * (y[0] - y[base + 0]));
    success            *= close(f[base + 1], -tau * yp[base + 1] + lag_count * (y[base + 0] - y[base + 1]));
    success            *= close(f[base + 2], -tau * yp[base + 2] + lag_count * (y[base + 1] - y[base + 2]));
    success            *= close(f[base + 3], -tau * yp[base + 3] + lag_count * (y[base + 2] - y[base + 3]));

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome delay_jacobian_matches_lag_chain()
  {
    using GridKit::Testing::TestStatus;

    const index_type  base  = 1;
    const scalar_type tau   = 4.0;
    const scalar_type n     = 4.0;
    const scalar_type alpha = 2.0;
    Delay             delay(makeData(tau, 1.0), scalarInput());

    std::vector<scalar_type> y(base + delay.size(), 0.0);
    std::vector<scalar_type> yp(base + delay.size(), 0.0);
    std::vector<scalar_type> f(base + delay.size(), 0.0);
    std::vector<index_type>  index(base + delay.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0] = 10.0;
    for (index_type i = 0; i < delay.size(); ++i)
    {
      y[base + i]  = static_cast<scalar_type>(i + 1);
      yp[base + i] = static_cast<scalar_type>(i) * 0.1;
    }

    delay.bind(y.data(), yp.data(), f.data(), index.data(), base);
    delay.setCoordinate(0.0, alpha);
    delay.evaluateJacobian();
    auto& J = delay.jacobian();

    TestStatus success  = true;
    success            *= (J.nnz() > 0);
    success            *= close(cooValue(J, base + 0, 0), n);
    success            *= close(cooValue(J, base + 0, base + 0), -n - tau * alpha);
    success            *= close(cooValue(J, base + 1, base + 0), n);
    success            *= close(cooValue(J, base + 1, base + 1), -n - tau * alpha);
    success            *= close(cooValue(J, base + 2, base + 1), n);
    success            *= close(cooValue(J, base + 2, base + 2), -n - tau * alpha);
    success            *= close(cooValue(J, base + 3, base + 2), n);
    success            *= close(cooValue(J, base + 3, base + 3), -n - tau * alpha);
    success            *= close(cooValue(J, base + 1, 0), 0.0);

    std::vector<bool> tag(base + delay.size(), false);
    delay.tagDifferentiable(tag);
    success *= !tag[0];
    for (index_type i = 0; i < delay.size(); ++i)
    {
      success *= tag[base + i];
    }

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += delay_rejects_invalid_contracts();
  result += delay_initializes_from_scalar_input();
  result += delay_residual_matches_lag_chain();
  result += delay_jacobian_matches_lag_chain();
  return result.summary();
}
