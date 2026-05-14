#pragma once

#ifndef GRIDKIT_ENABLE_ENZYME
#error "GridKit EMT requires Enzyme. Do not include EMT Jacobian support without GRIDKIT_ENABLE_ENZYME."
#endif

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/AutomaticDifferentiation/Enzyme/EnzymeDefinitions.hpp>
#include <GridKit/Constants.hpp>
#include <GridKit/LinearAlgebra/SparseMatrix/CsrMatrix.hpp>
#include <GridKit/Model/EMT/System/Events.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>
#include <GridKit/Model/EMT/System/Views.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    class JacobianMatrix
    {
    public:
      using Entry      = std::pair<IdxT, IdxT>;
      using CsrMatrixT = GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>;

      void build(IdxT size, std::vector<Entry> entries)
      {
        size_ = size;

        std::sort(entries.begin(), entries.end());
        entries.erase(std::unique(entries.begin(), entries.end()), entries.end());

        row_ptrs_.assign(static_cast<size_t>(size_) + 1, IdxT{0});
        for (const auto& [row, col] : entries)
        {
          if (row >= size_ || col >= size_)
          {
            throw std::out_of_range("EMT Jacobian pattern entry is out of range");
          }
          ++row_ptrs_[static_cast<size_t>(row + 1)];
        }
        for (IdxT row = 0; row < size_; ++row)
        {
          row_ptrs_[static_cast<size_t>(row + 1)] += row_ptrs_[static_cast<size_t>(row)];
        }

        cols_.assign(entries.size(), IdxT{0});
        values_.assign(entries.size(), RealT{0.0});

        std::vector<IdxT> next = row_ptrs_;
        for (const auto& [row, col] : entries)
        {
          cols_[static_cast<size_t>(next[static_cast<size_t>(row)]++)] = col;
        }

        csr_ = std::make_unique<CsrMatrixT>(size_, size_, static_cast<IdxT>(entries.size()));
        csr_->setDataPointers(row_ptrs_.data(), cols_.data(), values_.data(), GridKit::LinearAlgebra::memory::HOST);
      }

      void clearValues()
      {
        std::fill(values_.begin(), values_.end(), RealT{0.0});
      }

      void markUpdated()
      {
        if (csr_)
        {
          csr_->setUpdated(GridKit::LinearAlgebra::memory::HOST);
        }
      }

      IdxT slot(IdxT row, IdxT col) const
      {
        if (row >= size_ || col >= size_)
        {
          return INVALID_INDEX<IdxT>;
        }

        const auto row_index = static_cast<size_t>(row);
        const auto first     = cols_.begin()
                           + static_cast<typename std::vector<IdxT>::difference_type>(row_ptrs_[row_index]);
        const auto last = cols_.begin()
                          + static_cast<typename std::vector<IdxT>::difference_type>(row_ptrs_[row_index + 1]);
        const auto it = std::lower_bound(first, last, col);
        if (it == last || *it != col)
        {
          return INVALID_INDEX<IdxT>;
        }
        return static_cast<IdxT>(it - cols_.begin());
      }

      void addToSlot(IdxT slot, RealT value)
      {
        values_[static_cast<size_t>(slot)] += value;
      }

      IdxT size() const
      {
        return size_;
      }

      IdxT nnz() const
      {
        return static_cast<IdxT>(values_.size());
      }

      CsrMatrixT* matrix() const
      {
        return csr_.get();
      }

      const IdxT* rowPtrs() const
      {
        return row_ptrs_.data();
      }

      const IdxT* columns() const
      {
        return cols_.data();
      }

      RealT* values()
      {
        return values_.data();
      }

      const RealT* values() const
      {
        return values_.data();
      }

    private:
      IdxT                        size_{0};
      std::vector<IdxT>           row_ptrs_;
      std::vector<IdxT>           cols_;
      std::vector<RealT>          values_;
      std::unique_ptr<CsrMatrixT> csr_;
    };

    namespace Detail
    {
      template <class ScalarT, typename IdxT>
      class SingleEquationView
      {
      public:
        SingleEquationView(IdxT target, IdxT equation_count)
          : target_{target},
            equation_count_{equation_count}
        {
        }

        template <class ValueT>
        void set(IdxT local, ValueT value)
        {
          if (local == target_)
          {
            value_ = static_cast<ScalarT>(value);
          }
        }

        template <class ValueT>
        void add(IdxT local, ValueT value)
        {
          if (local == target_)
          {
            value_ += static_cast<ScalarT>(value);
          }
        }

        template <class ValueT, size_t N>
        void set(IdxT first, const std::array<ValueT, N>& values)
        {
          for (IdxT i = 0; i < static_cast<IdxT>(N); ++i)
          {
            set(first + i, values[static_cast<size_t>(i)]);
          }
        }

        template <class ValueT>
        void injectCurrent(IdxT terminal, const std::array<ValueT, 3>& current)
        {
          const IdxT first = equation_count_ + Layout<IdxT>::phases * terminal;
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            if (first + phase == target_)
            {
              value_ += static_cast<ScalarT>(current[static_cast<size_t>(phase)]);
            }
          }
        }

        template <class ValueT>
        void injectCurrent(IdxT terminal, std::initializer_list<ValueT> current)
        {
          if (current.size() != 3)
          {
            return;
          }

          const IdxT first = equation_count_ + Layout<IdxT>::phases * terminal;
          auto       value = current.begin();
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase, ++value)
          {
            if (first + phase == target_)
            {
              value_ += static_cast<ScalarT>(*value);
            }
          }
        }

        ScalarT value() const
        {
          return value_;
        }

      private:
        IdxT    target_{0};
        IdxT    equation_count_{0};
        ScalarT value_{0.0};
      };

      template <class ModelT, class ScalarT, typename IdxT>
      struct ScalarResidualKernel
      {
        static ScalarT eval(const ModelT*  model,
                            const ScalarT* local,
                            IdxT           target,
                            ScalarT        time)
        {
          constexpr IdxT variables = static_cast<IdxT>(ComponentTraits<ModelT>::variable_count);
          constexpr IdxT equations = static_cast<IdxT>(ComponentTraits<ModelT>::equation_count);
          constexpr IdxT terminals = static_cast<IdxT>(ComponentTraits<ModelT>::terminal_count);

          const IdxT     terminal_size = Layout<IdxT>::phases * terminals;
          const ScalarT* y             = local;
          const ScalarT* yp            = y + variables;
          const ScalarT* terminal_v    = yp + variables;
          const ScalarT* terminal_vp   = terminal_v + terminal_size;
          const ScalarT* inputs        = terminal_vp + terminal_size;

          LocalStateView<ScalarT, IdxT>     state(y, yp, terminal_v, terminal_vp, inputs, time);
          SingleEquationView<ScalarT, IdxT> equations_view(target, equations);
          model->residual(state, equations_view);
          return equations_view.value();
        }
      };
    } // namespace Detail

    template <class ScalarT, typename IdxT>
    class ComponentJacobianPlan
    {
    public:
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using Entry = typename JacobianMatrix<RealT, IdxT>::Entry;

      void configure(const Layout<IdxT>& layout, ComponentId id)
      {
        component_ = layout.component(id);

        const auto terminal_buses = layout.terminalBuses(id);
        terminal_buses_.assign(terminal_buses.begin(), terminal_buses.end());

        const auto input_variables = layout.inputVariables(id);
        input_variables_.assign(input_variables.begin(), input_variables.end());

        residual_count_ = component_.equation_count + Layout<IdxT>::phases * component_.terminal_count;
        terminal_size_  = Layout<IdxT>::phases * component_.terminal_count;
        active_count_   = IdxT{2} * component_.variable_count + IdxT{2} * terminal_size_ + component_.input_count;

        row_indices_.clear();
        row_indices_.reserve(static_cast<size_t>(residual_count_));
        for (IdxT local = 0; local < component_.equation_count; ++local)
        {
          row_indices_.push_back(component_.equation_offset + local);
        }
        for (IdxT terminal = 0; terminal < component_.terminal_count; ++terminal)
        {
          const IdxT bus = terminal_buses_[static_cast<size_t>(terminal)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            row_indices_.push_back(layout.busEquation(bus, phase));
          }
        }

        sources_.clear();
        column_indices_.clear();
        column_scales_.assign(static_cast<size_t>(active_count_), ScalarT{1.0});

        for (IdxT local = 0; local < component_.variable_count; ++local)
        {
          addSource(false, component_.variable_offset + local);
        }
        for (IdxT local = 0; local < component_.variable_count; ++local)
        {
          addSource(true, component_.variable_offset + local);
        }
        for (IdxT terminal = 0; terminal < component_.terminal_count; ++terminal)
        {
          const IdxT bus = terminal_buses_[static_cast<size_t>(terminal)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            addSource(false, layout.busVariable(bus, phase));
          }
        }
        for (IdxT terminal = 0; terminal < component_.terminal_count; ++terminal)
        {
          const IdxT bus = terminal_buses_[static_cast<size_t>(terminal)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            addSource(true, layout.busVariable(bus, phase));
          }
        }
        for (IdxT local = 0; local < component_.input_count; ++local)
        {
          addSource(false, input_variables_[static_cast<size_t>(local)]);
        }

        local_.assign(static_cast<size_t>(active_count_), ScalarT{0.0});
        seed_.assign(static_cast<size_t>(active_count_), ScalarT{0.0});
        nonzeros_.clear();
      }

      template <class ModelT>
      void discoverPattern(const ModelT&               model,
                           const std::vector<ScalarT>& y,
                           const std::vector<ScalarT>& yp,
                           ScalarT                     time,
                           std::vector<Entry>&         entries)
      {
        nonzeros_.clear();
        discoverPatternUnion(model, y, yp, time);
        flushPattern(entries);
      }

      template <class ModelT>
      void discoverPatternUnion(const ModelT&               model,
                                const std::vector<ScalarT>& y,
                                const std::vector<ScalarT>& yp,
                                ScalarT                     time)
      {
        if (active_count_ == IdxT{0} || residual_count_ == IdxT{0})
        {
          return;
        }

        updateLocalState(y, yp, ScalarT{1.0});
        auto mode_nonzeros = discoverLocalPattern(model, time);
        nonzeros_.insert(nonzeros_.end(), mode_nonzeros.begin(), mode_nonzeros.end());
        std::sort(nonzeros_.begin(), nonzeros_.end());
        nonzeros_.erase(std::unique(nonzeros_.begin(), nonzeros_.end()), nonzeros_.end());
      }

      void flushPattern(std::vector<Entry>& entries) const
      {
        for (const auto& nonzero : nonzeros_)
        {
          entries.emplace_back(row_indices_[static_cast<size_t>(nonzero.row)],
                               column_indices_[static_cast<size_t>(nonzero.source)]);
        }
      }

      void bindSlots(const JacobianMatrix<RealT, IdxT>& matrix)
      {
        for (auto& nonzero : nonzeros_)
        {
          nonzero.slot = matrix.slot(row_indices_[static_cast<size_t>(nonzero.row)],
                                     column_indices_[static_cast<size_t>(nonzero.source)]);
          if (nonzero.slot == INVALID_INDEX<IdxT>)
          {
            throw std::logic_error("EMT Jacobian slot map is missing a structural entry");
          }
        }
      }

      template <class ModelT>
      void evaluate(const ModelT&                model,
                    JacobianMatrix<RealT, IdxT>& matrix,
                    const std::vector<ScalarT>&  y,
                    const std::vector<ScalarT>&  yp,
                    ScalarT                      time,
                    RealT                        alpha)
      {
        if (active_count_ == IdxT{0} || residual_count_ == IdxT{0})
        {
          return;
        }

        updateLocalState(y, yp, static_cast<ScalarT>(alpha));
        for (const auto& nonzero : nonzeros_)
        {
          const ScalarT scale = column_scales_[static_cast<size_t>(nonzero.source)];
          const ScalarT value = scale * derivative(model, nonzero.row, nonzero.source, time);
          if (value == ScalarT{0.0})
          {
            continue;
          }

          matrix.addToSlot(nonzero.slot, static_cast<RealT>(value));
        }
      }

    private:
      struct Source
      {
        bool derivative{false};
        IdxT global{INVALID_INDEX<IdxT>};
      };

      struct Nonzero
      {
        IdxT row{0};
        IdxT source{0};
        IdxT slot{INVALID_INDEX<IdxT>};

        friend bool operator<(const Nonzero& lhs, const Nonzero& rhs)
        {
          return std::tie(lhs.row, lhs.source) < std::tie(rhs.row, rhs.source);
        }

        friend bool operator==(const Nonzero& lhs, const Nonzero& rhs)
        {
          return lhs.row == rhs.row && lhs.source == rhs.source;
        }
      };

      void addSource(bool derivative, IdxT global)
      {
        sources_.push_back({derivative, global});
        column_indices_.push_back(global);
      }

      void updateLocalState(const std::vector<ScalarT>& y,
                            const std::vector<ScalarT>& yp,
                            ScalarT                     alpha)
      {
        for (size_t i = 0; i < sources_.size(); ++i)
        {
          const auto& source = sources_[i];
          local_[i]          = source.derivative ? yp[static_cast<size_t>(source.global)]
                                                 : y[static_cast<size_t>(source.global)];
          column_scales_[i]  = source.derivative ? alpha : ScalarT{1.0};
        }
      }

      template <class ModelT>
      std::vector<Nonzero> discoverLocalPattern(const ModelT& model, ScalarT time)
      {
        using TrackingScalar = GridKit::DependencyTracking::Variable;

        std::vector<TrackingScalar> local(static_cast<size_t>(active_count_));
        for (IdxT source = 0; source < active_count_; ++source)
        {
          auto& value = local[static_cast<size_t>(source)];
          value       = static_cast<double>(local_[static_cast<size_t>(source)]);
          value.setVariableNumber(static_cast<size_t>(source));
        }

        std::vector<TrackingScalar> residual(static_cast<size_t>(residual_count_));
        TrackingScalar              tracking_time(static_cast<double>(time));

        TrackingScalar* y           = local.data();
        TrackingScalar* yp          = y + component_.variable_count;
        TrackingScalar* terminal_v  = yp + component_.variable_count;
        TrackingScalar* terminal_vp = terminal_v + terminal_size_;
        TrackingScalar* inputs      = terminal_vp + terminal_size_;

        Detail::LocalStateView<TrackingScalar, IdxT>    state(y,
                                                           yp,
                                                           terminal_v,
                                                           terminal_vp,
                                                           inputs,
                                                           tracking_time);
        Detail::LocalEquationView<TrackingScalar, IdxT> equations(residual.data(),
                                                                  component_.equation_count);
        model.residual(state, equations);

        std::vector<Nonzero> nonzeros;
        for (IdxT row = 0; row < residual_count_; ++row)
        {
          const auto& dependencies = residual[static_cast<size_t>(row)].getDependencies();
          for (const auto& [source, derivative] : dependencies)
          {
            if (derivative != 0.0)
            {
              nonzeros.push_back({row, static_cast<IdxT>(source), INVALID_INDEX<IdxT>});
            }
          }
        }

        std::sort(nonzeros.begin(), nonzeros.end());
        nonzeros.erase(std::unique(nonzeros.begin(), nonzeros.end()), nonzeros.end());
        return nonzeros;
      }

      template <class ModelT>
      ScalarT derivative(const ModelT& model, IdxT row, IdxT source, ScalarT time)
      {
        const auto source_index = static_cast<size_t>(source);
        seed_[source_index]     = ScalarT{1.0};
        const ScalarT value     = GridKit::Enzyme::Sparse::__enzyme_fwddiff<ScalarT>(
            (void*) Detail::ScalarResidualKernel<ModelT, ScalarT, IdxT>::eval,
            enzyme_const,
            &model,
            enzyme_dup,
            local_.data(),
            seed_.data(),
            enzyme_const,
            row,
            enzyme_const,
            time);
        seed_[source_index] = ScalarT{0.0};
        return value;
      }

      ComponentLayout<IdxT> component_;
      IdxT                  residual_count_{0};
      IdxT                  terminal_size_{0};
      IdxT                  active_count_{0};

      std::vector<IdxT>    terminal_buses_;
      std::vector<IdxT>    input_variables_;
      std::vector<IdxT>    row_indices_;
      std::vector<IdxT>    column_indices_;
      std::vector<Source>  sources_;
      std::vector<Nonzero> nonzeros_;

      std::vector<ScalarT> local_;
      std::vector<ScalarT> seed_;
      std::vector<ScalarT> column_scales_;
    };

    template <class ScalarT, typename IdxT>
    class JacobianPlan
    {
    public:
      using RealT  = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using Matrix = JacobianMatrix<RealT, IdxT>;
      using Entry  = typename Matrix::Entry;

      template <class NetworkT>
      void build(const NetworkT&             network,
                 const Layout<IdxT>&         layout,
                 const std::vector<ScalarT>& scratch_y,
                 const std::vector<ScalarT>& scratch_yp,
                 ScalarT                     scratch_time)
      {
        components_.clear();
        std::vector<Entry> entries;

        network.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT = std::decay_t<decltype(component)>;

              ComponentJacobianPlan<ScalarT, IdxT> plan;
              plan.configure(layout, id);
              ComponentStructuralModes<ComponentT>::visit(
                  component,
                  [&](const auto& structural_component)
                  {
                    plan.discoverPatternUnion(structural_component,
                                              scratch_y,
                                              scratch_yp,
                                              scratch_time);
                  });
              plan.flushPattern(entries);
              components_.push_back(std::move(plan));
            });

        matrix_.build(layout.size(), std::move(entries));
        for (auto& component : components_)
        {
          component.bindSlots(matrix_);
        }
      }

      template <class NetworkT>
      void evaluate(NetworkT&                   network,
                    const std::vector<ScalarT>& y,
                    const std::vector<ScalarT>& yp,
                    ScalarT                     time,
                    RealT                       alpha)
      {
        matrix_.clearValues();
        size_t plan_index = 0;
        network.components.forEach(
            [&](auto& component, ComponentId)
            {
              components_.at(plan_index).evaluate(component, matrix_, y, yp, time, alpha);
              ++plan_index;
            });
        matrix_.markUpdated();
      }

      Matrix& matrix()
      {
        return matrix_;
      }

      const Matrix& matrix() const
      {
        return matrix_;
      }

    private:
      Matrix                                            matrix_;
      std::vector<ComponentJacobianPlan<ScalarT, IdxT>> components_;
    };
  } // namespace EMT
} // namespace GridKit
