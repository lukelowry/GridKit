/**
 * @file StateSpace.hpp
 *
 * @brief Real-valued state-space realization of a factorized rational operator.
 *
 */

#pragma once

#include <array>
#include <span>
#include <vector>

#include <GridKit/Model/EMT/Element.hpp>
#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpaceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Rational
      {
        template <typename scalar_type, typename index_type>
        class StateSpace : public Element<scalar_type, index_type>
        {
        public:
          using ScalarT        = scalar_type;
          using IdxT           = index_type;
          using SignalT        = typename Element<ScalarT, IdxT>::SignalT;
          using SignalStorageT = typename Element<ScalarT, IdxT>::SignalStorageT;

          StateSpace(StateSpaceData<ScalarT, IdxT> data, SignalT input);

          IdxT size() const override
          {
            return layout_.n;
          }

          std::span<const SignalT> inputs() const override
          {
            inputs_[0] = input_;
            inputs_[1] = input_dot_;
            return inputs_;
          }

          SignalT out() const
          {
            return {base_ + layout_.out, layout_.N, 1};
          }

          int initialize() override;

          template <typename V>
          void evaluateResidualKernel(V* y, V* yp, V* u, V* f) const;

          void evaluateInternalResidual(ScalarT* y, ScalarT* yp, ScalarT* u, ScalarT* f) const;

          void tagDifferentiable(std::vector<bool>& tag) const override;

          int evaluateResidual() override;
          int evaluateJacobian() override;

        private:
          struct Layout
          {
            Layout(IdxT output_dim, IdxT input_dim, IdxT pole_count)
              : N(output_dim),
                K(input_dim),
                Q(pole_count),
                xr(0),
                xi(Q),
                out(2 * Q),
                n(out + N)
            {
            }

            const IdxT N;
            const IdxT K;
            const IdxT Q;
            const IdxT xr;
            const IdxT xi;
            const IdxT out;
            const IdxT n;
          };

          ScalarT d(IdxT i, IdxT k) const
          {
            return data_.D[static_cast<size_t>(i * layout_.K + k)];
          }

          ScalarT e(IdxT i, IdxT k) const
          {
            return data_.E[static_cast<size_t>(i * layout_.K + k)];
          }

          ScalarT cr(IdxT i, IdxT q) const
          {
            return Cr_[static_cast<size_t>(i * layout_.Q + q)];
          }

          ScalarT ci(IdxT i, IdxT q) const
          {
            return Ci_[static_cast<size_t>(i * layout_.Q + q)];
          }

          ScalarT br(IdxT q, IdxT k) const
          {
            return Br_[static_cast<size_t>(q * layout_.K + k)];
          }

          ScalarT bi(IdxT q, IdxT k) const
          {
            return Bi_[static_cast<size_t>(q * layout_.K + k)];
          }

          static void validateInput(SignalT input, IdxT K);

          void validateData() const;
          void deriveRealParameters();

          using Element<ScalarT, IdxT>::base_;
          using Element<ScalarT, IdxT>::y_;
          using Element<ScalarT, IdxT>::yp_;
          using Element<ScalarT, IdxT>::f_;
          using Element<ScalarT, IdxT>::state;
          using Element<ScalarT, IdxT>::derivative;
          using Element<ScalarT, IdxT>::slice;
          using Element<ScalarT, IdxT>::input;

          StateSpaceData<ScalarT, IdxT>  data_;
          SignalT                        input_;
          SignalT                        input_dot_;
          const Layout                   layout_;
          std::vector<ScalarT>           a_;
          std::vector<ScalarT>           omega_;
          std::vector<ScalarT>           Cr_;
          std::vector<ScalarT>           Ci_;
          std::vector<ScalarT>           Br_;
          std::vector<ScalarT>           Bi_;
          mutable std::array<SignalT, 2> inputs_{};
        };
      } // namespace Rational
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
