/**
 * @file Propagation.cpp
 *
 * @brief Initialization, residual, and mechanical child-Jacobian assembly for
 * the composite EMT Propagation operator.
 *
 */

#include "Propagation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename ScalarT, typename IdxT>
        Propagation<ScalarT, IdxT>::Propagation(DataT data, SignalT input)
          : data_(std::move(data)),
            input_(input),
            input_dot_(input),
            layout_(data_)
        {
          input_dot_.storage = SignalStorageT::Derivative;
          validateInput(input_, layout_.K);
        }

        template <typename ScalarT, typename IdxT>
        Propagation<ScalarT, IdxT>::Layout::Layout(const DataT& data)
        {
          Propagation<ScalarT, IdxT>::validateData(data);

          K          = data.input.K;
          M          = data.input.N;
          input      = 0;
          input_out  = 2 * data.input.Q;
          input_size = 2 * data.input.Q + data.input.N;

          delay.resize(static_cast<size_t>(M));
          delay_size.resize(static_cast<size_t>(M));

          IdxT offset = input + input_size;
          for (IdxT m = 0; m < M; ++m)
          {
            const auto n   = static_cast<size_t>(m);
            delay[n]       = offset;
            delay_size[n]  = Propagation<ScalarT, IdxT>::lagCount(data.tau[n], data.dt_min);
            offset        += delay_size[n];
          }

          z_mod   = offset;
          offset += M;

          output      = offset;
          output_out  = output + 2 * data.output.Q;
          output_size = 2 * data.output.Q + data.output.N;
          n           = output + output_size;
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::validateInput(SignalT input, IdxT K)
        {
          if (input.rows != K || input.cols != 1)
          {
            throw std::runtime_error("Propagation input signal must be a K-vector");
          }
          if (input.storage != SignalStorageT::State)
          {
            throw std::runtime_error("Propagation input signal must use state storage");
          }
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::validateData(const DataT& data)
        {
          StateSpaceT input_factor{data.input, SignalT{0, data.input.K, 1}};
          (void) input_factor;

          const IdxT K = data.input.K;
          const IdxT M = data.input.N;
          if (data.tau.size() != static_cast<size_t>(M))
          {
            throw std::runtime_error(
                "Propagation tau size must match input StateSpace output dimension");
          }

          StateSpaceT output_factor{data.output, SignalT{0, data.output.K, 1}};
          (void) output_factor;

          if (data.output.N != K)
          {
            throw std::runtime_error(
                "Propagation output StateSpace output dimension must match external input dimension");
          }
          if (data.output.K != M)
          {
            throw std::runtime_error(
                "Propagation output StateSpace input dimension must match modal dimension");
          }

          for (IdxT m = 0; m < M; ++m)
          {
            (void) lagCount(data.tau[static_cast<size_t>(m)], data.dt_min);
          }
        }

        template <typename ScalarT, typename IdxT>
        IdxT Propagation<ScalarT, IdxT>::lagCount(ScalarT delay, ScalarT dt_min)
        {
          if (!std::isfinite(static_cast<double>(delay)) || delay <= ScalarT{0.0})
          {
            throw std::runtime_error("Propagation requires positive finite delays");
          }
          if (!std::isfinite(static_cast<double>(dt_min)) || dt_min <= ScalarT{0.0})
          {
            throw std::runtime_error("Propagation requires a positive finite dt_min");
          }

          const ScalarT raw = std::floor(delay / dt_min);
          if (!std::isfinite(static_cast<double>(raw))
              || raw > static_cast<ScalarT>(std::numeric_limits<IdxT>::max()))
          {
            throw std::runtime_error("Propagation delay lag count is not representable");
          }

          return static_cast<IdxT>(std::max(ScalarT{1.0}, raw));
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::buildChildren()
        {
          input_factor_.emplace(data_.input, input_);

          delays_.clear();
          delays_.reserve(static_cast<size_t>(layout_.M));
          const SignalT modal = uMod();
          for (IdxT m = 0; m < layout_.M; ++m)
          {
            DelayData<ScalarT, IdxT> delay_data;
            delay_data.delay  = data_.tau[static_cast<size_t>(m)];
            delay_data.dt_min = data_.dt_min;
            delays_.emplace_back(delay_data, SignalT{modal.offset + m, 1, 1});
          }

          output_factor_.emplace(data_.output, zMod());

          children_.clear();
          children_.push_back(&*input_factor_);
          for (auto& delay : delays_)
          {
            children_.push_back(&delay);
          }
          children_.push_back(&*output_factor_);
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::bindChildren()
        {
          buildChildren();

          input_factor_->bind(gy_, gyp_, gf_, gindex_, base_ + layout_.input);
          for (IdxT m = 0; m < layout_.M; ++m)
          {
            delays_[static_cast<size_t>(m)].bind(
                gy_, gyp_, gf_, gindex_, base_ + layout_.delay[static_cast<size_t>(m)]);
          }
          output_factor_->bind(gy_, gyp_, gf_, gindex_, base_ + layout_.output);
          setChildCoordinate(coordinate_, alpha_);
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::setChildCoordinate(RealT coordinate, RealT alpha)
        {
          for (auto* child : children_)
          {
            child->setCoordinate(coordinate, alpha);
          }
        }

        template <typename ScalarT, typename IdxT>
        int Propagation<ScalarT, IdxT>::initialize()
        {
          input_factor_->initialize();

          for (auto& delay : delays_)
          {
            delay.initialize();
          }

          auto z  = state(layout_.z_mod, layout_.M);
          auto zd = derivative(layout_.z_mod, layout_.M);
          for (IdxT m = 0; m < layout_.M; ++m)
          {
            z[m]  = input(delayOut(m))[0];
            zd[m] = input(delayOutDot(m))[0];
          }

          output_factor_->initialize();
          return 0;
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::evaluateGatherResidual()
        {
          auto z  = state(layout_.z_mod, layout_.M);
          auto Fz = residual(layout_.z_mod, layout_.M);

          for (IdxT m = 0; m < layout_.M; ++m)
          {
            Fz[m] = -z[m] + input(delayOut(m))[0];
          }
        }

        template <typename ScalarT, typename IdxT>
        int Propagation<ScalarT, IdxT>::evaluateResidual()
        {
          for (auto* child : children_)
          {
            child->evaluateResidual();
          }
          evaluateGatherResidual();
          return 0;
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::appendGatherJacobian(std::vector<IdxT>&  rows,
                                                              std::vector<IdxT>&  cols,
                                                              std::vector<RealT>& vals) const
        {
          for (IdxT m = 0; m < layout_.M; ++m)
          {
            const IdxT row = index_[layout_.z_mod + m];
            const IdxT z   = index_[layout_.z_mod + m];
            const IdxT d   = inputIndices(delayOut(m))[0];

            rows.push_back(row);
            cols.push_back(z);
            vals.push_back(RealT{-1.0});

            rows.push_back(row);
            cols.push_back(d);
            vals.push_back(RealT{1.0});
          }
        }

        template <typename ScalarT, typename IdxT>
        int Propagation<ScalarT, IdxT>::evaluateJacobian()
        {
          std::vector<IdxT>  rows;
          std::vector<IdxT>  cols;
          std::vector<RealT> vals;

          for (auto* child : children_)
          {
            child->evaluateJacobian();
            auto [r, c, v] = child->jacobian().getEntryCopies();
            rows.insert(rows.end(), r.begin(), r.end());
            cols.insert(cols.end(), c.begin(), c.end());
            vals.insert(vals.end(), v.begin(), v.end());
          }

          appendGatherJacobian(rows, cols, vals);

          auto& J = this->jacobian();
          J.zeroMatrix();
          if (!vals.empty())
          {
            J.setValues(RealT{1.0},
                        rows.data(),
                        cols.data(),
                        vals.data(),
                        static_cast<IdxT>(vals.size()));
            J.deduplicate();
          }

          return 0;
        }

        template <typename ScalarT, typename IdxT>
        void Propagation<ScalarT, IdxT>::tagDifferentiable(std::vector<bool>& tag) const
        {
          for (auto* child : children_)
          {
            child->tagDifferentiable(tag);
          }
        }

        template class Propagation<double, long int>;
        template class Propagation<double, size_t>;
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit
