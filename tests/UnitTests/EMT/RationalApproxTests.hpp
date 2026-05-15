#pragma once

#include <array>
#include <span>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/EMT/Math/RationalApprox/RationalApprox.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class RealT>
    class EMTRationalApproxTests
    {
    public:
      using Data  = EMT::Math::RationalApproxData<RealT>;
      using Model = EMT::Math::RationalApprox<RealT>;

      TestOutcome countsAndValidation()
      {
        TestStatus success = true;

        auto data       = zeroData(2);
        data.real_poles = {-1.0, -2.0};
        data.real_residues.assign(2 * 4, RealT{0.0});
        data.pair_real = {-3.0};
        data.pair_imag = {4.0};
        data.pair_residue_real.assign(4, RealT{0.0});
        data.pair_residue_imag.assign(4, RealT{0.0});

        Model model(data);
        success *= (model.dimension() == 2);
        success *= (model.stateCount() == 8);
        success *= (model.equationCount() == 8);
        success *= (!model.hasDerivativeFeedthrough());

        auto bad_dimension       = zeroData(1);
        bad_dimension.dimension  = 0;
        success                 *= throws<std::invalid_argument>(
            [&]()
            {
              Model bad(bad_dimension);
            });

        auto bad_pole           = zeroData(1);
        bad_pole.real_poles     = {0.0};
        bad_pole.real_residues  = {0.0};
        success                *= throws<std::invalid_argument>(
            [&]()
            {
              Model bad(bad_pole);
            });

        return success.report(__func__);
      }

      TestOutcome directAndDerivativeOutput()
      {
        TestStatus success = true;

        auto data = zeroData(2);
        data.d    = {1.0, 2.0, 3.0, 4.0};
        data.e    = {0.5, 0.0, 0.0, 1.5};

        Model model(data);
        success *= model.hasDerivativeFeedthrough();

        std::vector<RealT>   y;
        std::vector<RealT>   yp;
        std::vector<RealT>   f;
        std::array<RealT, 2> u{2.0, 3.0};
        std::array<RealT, 2> up{4.0, 5.0};
        std::array<RealT, 2> z{};

        model.residual(std::span<const RealT>(y.data(), y.size()),
                       std::span<const RealT>(yp.data(), yp.size()),
                       std::span<RealT>(f.data(), f.size()),
                       u,
                       up,
                       z);

        success *= isEqual(z[0], RealT{10.0});
        success *= isEqual(z[1], RealT{25.5});

        return success.report(__func__);
      }

      TestOutcome realPoleResidual()
      {
        TestStatus success = true;

        auto data          = zeroData(2);
        data.real_poles    = {-2.0};
        data.real_residues = {5.0, 6.0, 7.0, 8.0};

        Model model(data);

        std::vector<RealT>   y{1.0, 2.0};
        std::vector<RealT>   yp{0.1, 0.2};
        std::vector<RealT>   f(2, RealT{0.0});
        std::array<RealT, 2> u{3.0, 4.0};
        std::array<RealT, 2> up{0.0, 0.0};
        std::array<RealT, 2> z{};

        model.residual(std::span<const RealT>(y.data(), y.size()),
                       std::span<const RealT>(yp.data(), yp.size()),
                       std::span<RealT>(f.data(), f.size()),
                       u,
                       up,
                       z);

        success *= isEqual(f[0], RealT{0.9});
        success *= isEqual(f[1], RealT{-0.2});
        success *= isEqual(z[0], RealT{17.0});
        success *= isEqual(z[1], RealT{23.0});

        return success.report(__func__);
      }

      TestOutcome complexPairResidual()
      {
        TestStatus success = true;

        auto data              = zeroData(1);
        data.pair_real         = {-1.0};
        data.pair_imag         = {2.0};
        data.pair_residue_real = {3.0};
        data.pair_residue_imag = {5.0};

        Model model(data);

        std::vector<RealT>   y{4.0, 6.0};
        std::vector<RealT>   yp{0.5, 0.75};
        std::vector<RealT>   f(2, RealT{0.0});
        std::array<RealT, 1> u{7.0};
        std::array<RealT, 1> up{0.0};
        std::array<RealT, 1> z{};

        model.residual(std::span<const RealT>(y.data(), y.size()),
                       std::span<const RealT>(yp.data(), yp.size()),
                       std::span<RealT>(f.data(), f.size()),
                       u,
                       up,
                       z);

        success *= isEqual(f[0], RealT{-9.5});
        success *= isEqual(f[1], RealT{1.25});
        success *= isEqual(z[0], RealT{-36.0});

        return success.report(__func__);
      }

      TestOutcome fullMatrixResidueOutput()
      {
        TestStatus success = true;

        auto data              = zeroData(2);
        data.real_poles        = {-1.0};
        data.real_residues     = {1.0, 2.0, 3.0, 4.0};
        data.pair_real         = {-3.0};
        data.pair_imag         = {4.0};
        data.pair_residue_real = {0.5, 1.0, 1.5, 2.0};
        data.pair_residue_imag = {2.0, 0.0, 0.0, 1.0};

        Model model(data);

        std::vector<RealT>   y{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        std::vector<RealT>   yp(6, RealT{0.0});
        std::vector<RealT>   f(6, RealT{0.0});
        std::array<RealT, 2> u{0.0, 0.0};
        std::array<RealT, 2> up{0.0, 0.0};
        std::array<RealT, 2> z{};

        model.residual(std::span<const RealT>(y.data(), y.size()),
                       std::span<const RealT>(yp.data(), yp.size()),
                       std::span<RealT>(f.data(), f.size()),
                       u,
                       up,
                       z);

        success *= isEqual(z[0], RealT{-4.0});
        success *= isEqual(z[1], RealT{24.0});

        return success.report(__func__);
      }

      TestOutcome initialization()
      {
        TestStatus success = true;

        auto data              = zeroData(1);
        data.real_poles        = {-2.0};
        data.real_residues     = {0.0};
        data.pair_real         = {-1.0};
        data.pair_imag         = {2.0};
        data.pair_residue_real = {0.0};
        data.pair_residue_imag = {0.0};

        Model model(data);

        std::vector<RealT>   y(3, RealT{0.0});
        std::vector<RealT>   yp(3, RealT{0.0});
        std::array<RealT, 1> u0{4.0};
        std::array<RealT, 1> up0{6.0};

        model.initialize(std::span<RealT>(y.data(), y.size()),
                         std::span<RealT>(yp.data(), yp.size()),
                         u0,
                         up0);

        success *= isEqual(y[0], RealT{0.5});
        success *= isEqual(yp[0], RealT{3.0});

        std::array<RealT, 1> complex_u0{5.0};
        std::array<RealT, 1> complex_up0{0.0};
        model.initialize(std::span<RealT>(y.data(), y.size()),
                         std::span<RealT>(yp.data(), yp.size()),
                         complex_u0,
                         complex_up0);

        success *= isEqual(y[1], RealT{1.0});
        success *= isEqual(y[2], RealT{2.0});
        success *= isEqual(yp[1], RealT{0.0});
        success *= isEqual(yp[2], RealT{0.0});

        return success.report(__func__);
      }

      TestOutcome independentApplications()
      {
        TestStatus success = true;

        auto data          = zeroData(1);
        data.real_poles    = {-1.0};
        data.real_residues = {2.0};

        Model             model(data);
        const std::size_t n = model.stateCount();

        std::vector<RealT>   y{1.0, 10.0};
        std::vector<RealT>   yp{0.0, 0.0};
        std::vector<RealT>   f(2, RealT{0.0});
        std::array<RealT, 1> u{0.0};
        std::array<RealT, 1> up{0.0};
        std::array<RealT, 1> z0{};
        std::array<RealT, 1> z1{};

        std::span<const RealT> y_span(y.data(), y.size());
        std::span<const RealT> yp_span(yp.data(), yp.size());
        std::span<RealT>       f_span(f.data(), f.size());

        model.residual(y_span.subspan(0, n), yp_span.subspan(0, n), f_span.subspan(0, n), u, up, z0);
        model.residual(y_span.subspan(n, n), yp_span.subspan(n, n), f_span.subspan(n, n), u, up, z1);

        success *= isEqual(f[0], RealT{-1.0});
        success *= isEqual(f[1], RealT{-10.0});
        success *= isEqual(z0[0], RealT{2.0});
        success *= isEqual(z1[0], RealT{20.0});

        return success.report(__func__);
      }

    private:
      static Data zeroData(std::size_t dimension)
      {
        Data data;
        data.dimension = dimension;
        data.d.assign(dimension * dimension, RealT{0.0});
        data.e.assign(dimension * dimension, RealT{0.0});
        return data;
      }
    };
  } // namespace Testing
} // namespace GridKit
