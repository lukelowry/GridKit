/**
 * @file Delay.hpp
 *
 * @brief Scalar lag-chain approximation of a transport delay.
 *
 */

#pragma once

#include <array>
#include <span>
#include <vector>

#include <GridKit/Model/EMT/Element.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Delay/DelayData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename scalar_type, typename index_type>
        class Delay : public Element<scalar_type, index_type>
        {
        public:
          using ScalarT        = scalar_type;
          using IdxT           = index_type;
          using SignalT        = typename Element<ScalarT, IdxT>::SignalT;
          using SignalStorageT = typename Element<ScalarT, IdxT>::SignalStorageT;

          Delay(DelayData<ScalarT, IdxT> data, SignalT input);

          IdxT size() const override
          {
            return layout_.n;
          }

          std::span<const SignalT> inputs() const override
          {
            inputs_[0] = input_;
            return inputs_;
          }

          SignalT out() const
          {
            return {base_ + layout_.last(), 1, 1};
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
            explicit Layout(IdxT lag_count)
              : x(0),
                n(lag_count)
            {
            }

            IdxT last() const
            {
              return n - 1;
            }

            const IdxT x;
            const IdxT n;
          };

          static IdxT lagCount(const DelayData<ScalarT, IdxT>& data);
          static void validateInput(SignalT input);

          using Element<ScalarT, IdxT>::base_;
          using Element<ScalarT, IdxT>::y_;
          using Element<ScalarT, IdxT>::yp_;
          using Element<ScalarT, IdxT>::f_;
          using Element<ScalarT, IdxT>::state;
          using Element<ScalarT, IdxT>::derivative;
          using Element<ScalarT, IdxT>::slice;
          using Element<ScalarT, IdxT>::input;

          DelayData<ScalarT, IdxT>       data_;
          SignalT                        input_;
          const Layout                   layout_;
          const ScalarT                  n_;
          mutable std::array<SignalT, 1> inputs_{};
        };
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
