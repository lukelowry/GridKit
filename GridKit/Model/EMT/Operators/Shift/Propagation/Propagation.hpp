/**
 * @file Propagation.hpp
 *
 * @brief Composite EMT propagation operator built from StateSpace, Delay, and
 * a temporary modal gather state.
 *
 */

#pragma once

#include <array>
#include <optional>
#include <span>
#include <vector>

#include <GridKit/Model/EMT/Element.hpp>
#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpace.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Delay/Delay.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Propagation/PropagationData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename scalar_type, typename index_type>
        class Propagation : public Element<scalar_type, index_type>
        {
        public:
          using ScalarT        = scalar_type;
          using IdxT           = index_type;
          using BaseT          = Element<ScalarT, IdxT>;
          using RealT          = typename BaseT::RealT;
          using SignalT        = typename BaseT::SignalT;
          using SignalStorageT = typename BaseT::SignalStorageT;
          using DataT          = PropagationData<ScalarT, IdxT>;
          using StateSpaceT    = Rational::StateSpace<ScalarT, IdxT>;
          using DelayT         = Delay<ScalarT, IdxT>;
          using ElementT       = Element<ScalarT, IdxT>;

          Propagation(DataT data, SignalT input);

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

          SignalT uMod() const
          {
            return {base_ + layout_.input_out, layout_.M, 1};
          }

          SignalT zMod() const
          {
            return {base_ + layout_.z_mod, layout_.M, 1};
          }

          SignalT out() const
          {
            return {base_ + layout_.output_out, layout_.K, 1};
          }

          int initialize() override;

          void tagDifferentiable(std::vector<bool>& tag) const override;

          int evaluateResidual() override;
          int evaluateJacobian() override;

        private:
          struct Layout
          {
            explicit Layout(const DataT& data);

            IdxT              K{0};
            IdxT              M{0};
            IdxT              input{0};
            IdxT              input_out{0};
            IdxT              input_size{0};
            std::vector<IdxT> delay;
            std::vector<IdxT> delay_size;
            IdxT              z_mod{0};
            IdxT              output{0};
            IdxT              output_out{0};
            IdxT              output_size{0};
            IdxT              n{0};
          };

          static void validateInput(SignalT input, IdxT K);
          static void validateData(const DataT& data);
          static IdxT lagCount(ScalarT delay, ScalarT dt_min);

          SignalT delayOut(IdxT m) const
          {
            const auto n = static_cast<size_t>(m);
            return {base_ + layout_.delay[n] + layout_.delay_size[n] - 1, 1, 1};
          }

          SignalT delayOutDot(IdxT m) const
          {
            SignalT signal = delayOut(m);
            signal.storage = SignalStorageT::Derivative;
            return signal;
          }

          void bindChildren() override;
          void setChildCoordinate(RealT coordinate, RealT alpha) override;
          void buildChildren();
          void evaluateGatherResidual();
          void appendGatherJacobian(std::vector<IdxT>&  rows,
                                    std::vector<IdxT>&  cols,
                                    std::vector<RealT>& vals) const;

          using BaseT::alpha_;
          using BaseT::base_;
          using BaseT::coordinate_;
          using BaseT::derivative;
          using BaseT::f_;
          using BaseT::gf_;
          using BaseT::gindex_;
          using BaseT::gy_;
          using BaseT::gyp_;
          using BaseT::index_;
          using BaseT::input;
          using BaseT::inputIndices;
          using BaseT::residual;
          using BaseT::state;
          using BaseT::y_;
          using BaseT::yp_;

          DataT   data_;
          SignalT input_;
          SignalT input_dot_;
          Layout  layout_;

          std::optional<StateSpaceT> input_factor_;
          std::vector<DelayT>        delays_;
          std::optional<StateSpaceT> output_factor_;
          std::vector<ElementT*>     children_;

          mutable std::array<SignalT, 2> inputs_{};
        };
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
