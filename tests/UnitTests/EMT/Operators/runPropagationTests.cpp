#include <cmath>
#include <complex>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/EMT/Operators/Shift/Propagation/Propagation.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace
{
  using scalar_type     = double;
  using index_type      = size_t;
  using Propagation     = GridKit::EMT::Operators::Shift::Propagation<scalar_type, index_type>;
  using PropagationData = GridKit::EMT::Operators::Shift::PropagationData<scalar_type, index_type>;
  using StateSpaceData  = GridKit::EMT::Operators::Rational::StateSpaceData<scalar_type, index_type>;
  using Signal          = Propagation::SignalT;
  using Storage         = Propagation::SignalStorageT;
  using Complex         = std::complex<scalar_type>;

  constexpr index_type input_xr   = 0;
  constexpr index_type input_xi   = 1;
  constexpr index_type input_out  = 2;
  constexpr index_type delay0     = 4;
  constexpr index_type delay1     = 6;
  constexpr index_type z_mod      = 9;
  constexpr index_type output_xr  = 11;
  constexpr index_type output_xi  = 12;
  constexpr index_type output_out = 13;
  constexpr index_type total_size = 15;

  bool close(scalar_type value, scalar_type ref, scalar_type tol = 1.0e-10)
  {
    return std::abs(value - ref) <= tol * (1.0 + std::abs(ref));
  }

  Signal vectorInput(index_type offset = 0)
  {
    return {offset, 2, 1};
  }

  StateSpaceData makeStateSpaceData(index_type N, index_type K)
  {
    StateSpaceData data;
    data.N     = N;
    data.K     = K;
    data.Q     = 1;
    data.poles = {Complex{-2.0, 0.0}};

    data.D.reserve(static_cast<size_t>(N * K));
    data.E.reserve(static_cast<size_t>(N * K));
    for (index_type i = 0; i < N; ++i)
    {
      for (index_type k = 0; k < K; ++k)
      {
        data.D.push_back(0.5 + 0.25 * static_cast<scalar_type>(i)
                         - 0.1 * static_cast<scalar_type>(k));
        data.E.push_back(0.05 * static_cast<scalar_type>((i + 1) * (k + 1)));
      }
    }

    data.C.reserve(static_cast<size_t>(N));
    for (index_type i = 0; i < N; ++i)
    {
      data.C.push_back(Complex{0.6 - 0.2 * static_cast<scalar_type>(i), 0.0});
    }

    data.B.reserve(static_cast<size_t>(K));
    for (index_type k = 0; k < K; ++k)
    {
      data.B.push_back(Complex{0.2 + 0.1 * static_cast<scalar_type>(k), 0.0});
    }

    return data;
  }

  PropagationData makeData()
  {
    PropagationData data;
    data.input  = makeStateSpaceData(2, 2);
    data.tau    = {2.0, 3.0};
    data.dt_min = 1.0;
    data.output = makeStateSpaceData(2, 2);
    return data;
  }

  scalar_type cooValue(Propagation::MatrixT& J, index_type row, index_type col)
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

  void bind(Propagation&              model,
            std::vector<scalar_type>& y,
            std::vector<scalar_type>& yp,
            std::vector<scalar_type>& f,
            std::vector<index_type>&  index,
            index_type                base)
  {
    model.bind(y.data(), yp.data(), f.data(), index.data(), base);
  }

  scalar_type stateSpaceXrResidual(const StateSpaceData&           data,
                                   const std::vector<scalar_type>& y,
                                   const std::vector<scalar_type>& yp,
                                   index_type                      state_offset,
                                   index_type                      input_offset)
  {
    scalar_type Bu = 0.0;
    for (index_type k = 0; k < data.K; ++k)
    {
      Bu += data.B[static_cast<size_t>(k)].real() * y[input_offset + k];
    }

    return -yp[state_offset] + data.poles[0].real() * y[state_offset] + Bu;
  }

  scalar_type stateSpaceXiResidual(const StateSpaceData&           data,
                                   const std::vector<scalar_type>& y,
                                   const std::vector<scalar_type>& yp,
                                   index_type                      state_offset)
  {
    return -yp[state_offset] + data.poles[0].real() * y[state_offset];
  }

  scalar_type stateSpaceOutResidual(const StateSpaceData&           data,
                                    const std::vector<scalar_type>& y,
                                    const std::vector<scalar_type>& yp,
                                    index_type                      state_offset,
                                    index_type                      out_offset,
                                    index_type                      input_offset,
                                    index_type                      i)
  {
    scalar_type value = -y[out_offset + i];
    for (index_type k = 0; k < data.K; ++k)
    {
      const auto nk  = static_cast<size_t>(i * data.K + k);
      value         += data.D[nk] * y[input_offset + k] + data.E[nk] * yp[input_offset + k];
    }
    value += data.C[static_cast<size_t>(i)].real() * y[state_offset];
    return value;
  }

  GridKit::Testing::TestOutcome propagation_rejects_invalid_contracts()
  {
    using GridKit::Testing::TestStatus;

    TestStatus success = true;

    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Propagation model(makeData(), Signal{0, 1, 1});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Propagation model(makeData(), Signal{0, 2, 2});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          Propagation model(makeData(), Signal{0, 2, 1, Storage::Derivative});
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data = makeData();
          data.tau.clear();
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.tau[0] = 0.0;
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.tau[0] = std::numeric_limits<scalar_type>::infinity();
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.dt_min = 0.0;
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.dt_min = std::numeric_limits<scalar_type>::quiet_NaN();
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.output = makeStateSpaceData(1, 2);
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data   = makeData();
          data.output = makeStateSpaceData(2, 1);
          Propagation model(data, vectorInput());
        });
    success *= GridKit::Testing::throws<std::runtime_error>(
        []()
        {
          auto data  = makeData();
          data.input = makeStateSpaceData(2, 1);
          Propagation model(data, vectorInput());
        });

    Propagation model(makeData(), vectorInput());
    success *= (model.size() == total_size);

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome propagation_initializes_child_chain_and_gather()
  {
    using GridKit::Testing::TestStatus;

    const index_type base = 3;
    Propagation      model(makeData(), vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0]  = 1.5;
    y[1]  = -0.75;
    yp[0] = 0.2;
    yp[1] = -0.1;

    bind(model, y, yp, f, index, base);
    model.initialize();

    auto       inputs = model.inputs();
    const auto u_mod  = model.uMod();
    const auto z      = model.zMod();
    const auto out    = model.out();

    TestStatus success  = true;
    success            *= (model.size() == total_size);
    success            *= (inputs.size() == 2);
    success            *= (inputs[0].offset == 0);
    success            *= (inputs[0].rows == 2);
    success            *= (inputs[0].storage == Storage::State);
    success            *= (inputs[1].offset == 0);
    success            *= (inputs[1].rows == 2);
    success            *= (inputs[1].storage == Storage::Derivative);
    success            *= (u_mod.offset == base + input_out);
    success            *= (u_mod.rows == 2);
    success            *= (z.offset == base + z_mod);
    success            *= (z.rows == 2);
    success            *= (out.offset == base + output_out);
    success            *= (out.rows == 2);
    success            *= (out.cols == 1);
    success            *= (out.storage == Storage::State);

    success *= close(y[base + delay0 + 0], y[base + input_out + 0]);
    success *= close(y[base + delay0 + 1], y[base + input_out + 0]);
    success *= close(y[base + delay1 + 0], y[base + input_out + 1]);
    success *= close(y[base + delay1 + 2], y[base + input_out + 1]);
    success *= close(y[base + z_mod + 0], y[base + delay0 + 1]);
    success *= close(y[base + z_mod + 1], y[base + delay1 + 2]);
    success *= close(yp[base + z_mod + 0], yp[base + delay0 + 1]);
    success *= close(yp[base + z_mod + 1], yp[base + delay1 + 2]);

    model.evaluateResidual();
    for (index_type i = 0; i < model.size(); ++i)
    {
      success *= close(f[base + i], 0.0, 1.0e-9);
    }

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome propagation_residual_matches_children_and_gather()
  {
    using GridKit::Testing::TestStatus;

    const auto       data = makeData();
    const index_type base = 2;
    Propagation      model(data, vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    y[0]  = -1.0;
    y[1]  = 2.0;
    yp[0] = 0.25;
    yp[1] = -0.5;
    for (index_type i = 0; i < model.size(); ++i)
    {
      y[base + i]  = 0.5 + 0.25 * static_cast<scalar_type>(i);
      yp[base + i] = -0.2 + 0.1 * static_cast<scalar_type>(i);
    }

    bind(model, y, yp, f, index, base);
    model.evaluateResidual();

    TestStatus success  = true;
    success            *= close(f[base + input_xr],
                     stateSpaceXrResidual(data.input, y, yp, base + input_xr, 0));
    success            *= close(f[base + input_xi],
                     stateSpaceXiResidual(data.input, y, yp, base + input_xi));
    success            *= close(f[base + input_out],
                     stateSpaceOutResidual(data.input, y, yp, base + input_xr, base + input_out, 0, 0));
    success            *= close(f[base + input_out + 1],
                     stateSpaceOutResidual(data.input, y, yp, base + input_xr, base + input_out, 0, 1));

    success *= close(f[base + delay0 + 0],
                     -2.0 * yp[base + delay0 + 0]
                         + 2.0 * (y[base + input_out + 0] - y[base + delay0 + 0]));
    success *= close(f[base + delay0 + 1],
                     -2.0 * yp[base + delay0 + 1]
                         + 2.0 * (y[base + delay0 + 0] - y[base + delay0 + 1]));
    success *= close(f[base + delay1 + 0],
                     -3.0 * yp[base + delay1 + 0]
                         + 3.0 * (y[base + input_out + 1] - y[base + delay1 + 0]));
    success *= close(f[base + delay1 + 2],
                     -3.0 * yp[base + delay1 + 2]
                         + 3.0 * (y[base + delay1 + 1] - y[base + delay1 + 2]));

    success *= close(f[base + z_mod + 0], -y[base + z_mod + 0] + y[base + delay0 + 1]);
    success *= close(f[base + z_mod + 1], -y[base + z_mod + 1] + y[base + delay1 + 2]);

    success *= close(f[base + output_xr],
                     stateSpaceXrResidual(data.output, y, yp, base + output_xr, base + z_mod));
    success *= close(f[base + output_xi],
                     stateSpaceXiResidual(data.output, y, yp, base + output_xi));
    success *= close(f[base + output_out],
                     stateSpaceOutResidual(
                         data.output, y, yp, base + output_xr, base + output_out, base + z_mod, 0));
    success *= close(f[base + output_out + 1],
                     stateSpaceOutResidual(
                         data.output, y, yp, base + output_xr, base + output_out, base + z_mod, 1));

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome propagation_jacobian_and_tags_match_composite_contract()
  {
    using GridKit::Testing::TestStatus;

    const auto        data  = makeData();
    const index_type  base  = 2;
    const scalar_type alpha = 5.0;
    Propagation       model(data, vectorInput());

    std::vector<scalar_type> y(base + model.size(), 0.0);
    std::vector<scalar_type> yp(base + model.size(), 0.0);
    std::vector<scalar_type> f(base + model.size(), 0.0);
    std::vector<index_type>  index(base + model.size(), 0);
    std::iota(index.begin(), index.end(), index_type{0});

    bind(model, y, yp, f, index, base);
    model.setCoordinate(0.0, alpha);
    model.evaluateJacobian();
    auto& J = model.jacobian();

    TestStatus success  = true;
    success            *= (J.nnz() > 0);
    success            *= close(cooValue(J, base + input_out, 0), data.input.D[0] + alpha * data.input.E[0]);
    success            *= close(cooValue(J, base + delay0, base + input_out), 2.0);
    success            *= close(cooValue(J, base + delay1, base + input_out + 1), 3.0);
    success            *= close(cooValue(J, base + z_mod, base + z_mod), -1.0);
    success            *= close(cooValue(J, base + z_mod, base + delay0 + 1), 1.0);
    success            *= close(cooValue(J, base + z_mod + 1, base + z_mod + 1), -1.0);
    success            *= close(cooValue(J, base + z_mod + 1, base + delay1 + 2), 1.0);
    success            *= close(cooValue(J, base + output_out, base + z_mod),
                     data.output.D[0] + alpha * data.output.E[0]);
    success            *= close(cooValue(J, base + output_out, base + z_mod + 1),
                     data.output.D[1] + alpha * data.output.E[1]);

    std::vector<bool> tag(base + model.size(), false);
    model.tagDifferentiable(tag);

    success *= !tag[0];
    success *= !tag[1];
    success *= tag[base + input_xr];
    success *= tag[base + input_xi];
    success *= !tag[base + input_out];
    success *= !tag[base + input_out + 1];
    for (index_type i = 0; i < 2; ++i)
    {
      success *= tag[base + delay0 + i];
    }
    for (index_type i = 0; i < 3; ++i)
    {
      success *= tag[base + delay1 + i];
    }
    success *= !tag[base + z_mod];
    success *= !tag[base + z_mod + 1];
    success *= tag[base + output_xr];
    success *= tag[base + output_xi];
    success *= !tag[base + output_out];
    success *= !tag[base + output_out + 1];

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += propagation_rejects_invalid_contracts();
  result += propagation_initializes_child_chain_and_gather();
  result += propagation_residual_matches_children_and_gather();
  result += propagation_jacobian_and_tags_match_composite_contract();
  return result.summary();
}
