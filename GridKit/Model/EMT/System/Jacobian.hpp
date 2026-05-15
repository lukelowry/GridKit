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
      template <class ModelT, class ScalarT, typename IdxT>
      struct VectorResidualKernel
      {
        static void eval(const ModelT*  model,
                         const ScalarT* local,
                         ScalarT*       residual,
                         ScalarT        time,
                         IdxT           variables,
                         IdxT           equations,
                         IdxT           ports)
        {
          const IdxT     port_size   = Layout<IdxT>::phases * ports;
          const ScalarT* y           = local;
          const ScalarT* yp          = y + variables;
          const ScalarT* port_v      = yp + variables;
          const ScalarT* port_vp     = port_v + port_size;
          const ScalarT* input_ports = port_vp + port_size;

          for (IdxT row = 0; row < equations + port_size; ++row)
          {
            residual[static_cast<size_t>(row)] = ScalarT{0.0};
          }

          LocalStateView<ScalarT, IdxT>    state(y, yp, port_v, port_vp, input_ports, time);
          LocalEquationView<ScalarT, IdxT> equations_view(residual, equations);
          model->residual(state, equations_view);
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

        const auto port_buses = layout.portBuses(id);
        port_buses_.assign(port_buses.begin(), port_buses.end());

        const auto input_port_variables = layout.inputPortVariables(id);
        input_port_variables_.assign(input_port_variables.begin(), input_port_variables.end());

        residual_count_ = component_.equation_count + Layout<IdxT>::phases * component_.electrical_port_count;
        port_size_      = Layout<IdxT>::phases * component_.electrical_port_count;
        active_count_   = IdxT{2} * component_.variable_count + IdxT{2} * port_size_ + component_.input_port_count;

        row_indices_.clear();
        row_indices_.reserve(static_cast<size_t>(residual_count_));
        for (IdxT local = 0; local < component_.equation_count; ++local)
        {
          row_indices_.push_back(component_.equation_offset + local);
        }
        for (IdxT port = 0; port < component_.electrical_port_count; ++port)
        {
          const IdxT bus = port_buses_[static_cast<size_t>(port)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            row_indices_.push_back(layout.busEquation(bus, phase));
          }
        }

        sources_.clear();

        for (IdxT local = 0; local < component_.variable_count; ++local)
        {
          addSource(false, component_.variable_offset + local);
        }
        for (IdxT local = 0; local < component_.variable_count; ++local)
        {
          addSource(true, component_.variable_offset + local);
        }
        for (IdxT port = 0; port < component_.electrical_port_count; ++port)
        {
          const IdxT bus = port_buses_[static_cast<size_t>(port)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            addSource(false, layout.busVariable(bus, phase));
          }
        }
        for (IdxT port = 0; port < component_.electrical_port_count; ++port)
        {
          const IdxT bus = port_buses_[static_cast<size_t>(port)];
          for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
          {
            addSource(true, layout.busVariable(bus, phase));
          }
        }
        for (IdxT local = 0; local < component_.input_port_count; ++local)
        {
          addSource(false, input_port_variables_[static_cast<size_t>(local)]);
        }

        local_.assign(static_cast<size_t>(active_count_), ScalarT{0.0});
        seed_.assign(static_cast<size_t>(active_count_), ScalarT{0.0});
        residual_.assign(static_cast<size_t>(residual_count_), ScalarT{0.0});
        residual_dot_.assign(static_cast<size_t>(residual_count_), ScalarT{0.0});
        nonzeros_.clear();
        coefficients_valid_ = false;
        coefficient_update_ = ComponentJacobianCoefficientUpdate::PerEvaluation;
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
        constexpr bool keep_zero_dependencies =
            ComponentJacobianTraits<ModelT>::form == ComponentJacobianForm::General
            || (ComponentJacobianTraits<ModelT>::form == ComponentJacobianForm::Affine
                && ComponentJacobianTraits<ModelT>::coefficient_update
                       != ComponentJacobianCoefficientUpdate::Static);
        if constexpr (ComponentJacobianTraits<ModelT>::form == ComponentJacobianForm::Affine)
        {
          coefficient_update_ = ComponentJacobianTraits<ModelT>::coefficient_update;
        }
        auto mode_nonzeros = discoverLocalPattern(model, time, keep_zero_dependencies);
        nonzeros_.insert(nonzeros_.end(), mode_nonzeros.begin(), mode_nonzeros.end());
        std::sort(nonzeros_.begin(), nonzeros_.end());
        nonzeros_.erase(std::unique(nonzeros_.begin(), nonzeros_.end()), nonzeros_.end());
      }

      void flushPattern(std::vector<Entry>& entries) const
      {
        for (const auto& nonzero : nonzeros_)
        {
          entries.emplace_back(row_indices_[static_cast<size_t>(nonzero.row)],
                               sources_[static_cast<size_t>(nonzero.source)].global);
        }
      }

      void bindSlots(const JacobianMatrix<RealT, IdxT>& matrix)
      {
        for (auto& source : sources_)
        {
          source.entries.clear();
        }

        for (auto& nonzero : nonzeros_)
        {
          nonzero.slot = matrix.slot(row_indices_[static_cast<size_t>(nonzero.row)],
                                     sources_[static_cast<size_t>(nonzero.source)].global);
          if (nonzero.slot == INVALID_INDEX<IdxT>)
          {
            throw std::logic_error("EMT Jacobian slot map is missing a structural entry");
          }

          sources_[static_cast<size_t>(nonzero.source)].entries.push_back(
              {nonzero.row, nonzero.slot, RealT{0.0}});
        }
      }

      void invalidateCachedCoefficients()
      {
        if (coefficient_update_ != ComponentJacobianCoefficientUpdate::Static)
        {
          coefficients_valid_ = false;
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
        if constexpr (ComponentJacobianTraits<ModelT>::form == ComponentJacobianForm::Affine)
        {
          evaluateAffine(model, matrix, time, alpha);
        }
        else
        {
          evaluateGeneral(model, matrix, time, alpha);
        }
      }

    private:
      struct Source
      {
        bool derivative{false};
        IdxT global{INVALID_INDEX<IdxT>};
        IdxT active{INVALID_INDEX<IdxT>};

        struct Entry
        {
          IdxT  local_row{0};
          IdxT  slot{INVALID_INDEX<IdxT>};
          RealT coefficient{0.0};
        };

        std::vector<Entry> entries;
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
        const IdxT active = static_cast<IdxT>(sources_.size());
        sources_.push_back({derivative, global, active, {}});
      }

      void updateLocalState(const std::vector<ScalarT>& y,
                            const std::vector<ScalarT>& yp,
                            ScalarT                     alpha)
      {
        (void) alpha;
        for (size_t i = 0; i < sources_.size(); ++i)
        {
          const auto& source = sources_[i];
          local_[i]          = source.derivative ? yp[static_cast<size_t>(source.global)]
                                                 : y[static_cast<size_t>(source.global)];
        }
      }

      template <class ModelT>
      std::vector<Nonzero> discoverLocalPattern(const ModelT& model,
                                                ScalarT       time,
                                                bool          keep_zero_dependencies)
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
        TrackingScalar* port_v      = yp + component_.variable_count;
        TrackingScalar* port_vp     = port_v + port_size_;
        TrackingScalar* input_ports = port_vp + port_size_;

        Detail::LocalStateView<TrackingScalar, IdxT>    state(y,
                                                           yp,
                                                           port_v,
                                                           port_vp,
                                                           input_ports,
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
            if (keep_zero_dependencies || derivative != 0.0)
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
      void refreshAffineCoefficients(const ModelT& model, ScalarT time)
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
        TrackingScalar* port_v      = yp + component_.variable_count;
        TrackingScalar* port_vp     = port_v + port_size_;
        TrackingScalar* input_ports = port_vp + port_size_;

        Detail::LocalStateView<TrackingScalar, IdxT>    state(y,
                                                           yp,
                                                           port_v,
                                                           port_vp,
                                                           input_ports,
                                                           tracking_time);
        Detail::LocalEquationView<TrackingScalar, IdxT> equations(residual.data(),
                                                                  component_.equation_count);
        model.residual(state, equations);

        for (auto& source : sources_)
        {
          for (auto& entry : source.entries)
          {
            const auto& dependencies = residual[static_cast<size_t>(entry.local_row)].getDependencies();
            const auto  it           = dependencies.find(static_cast<size_t>(source.active));
            entry.coefficient        = it == dependencies.end() ? RealT{0.0}
                                                                : static_cast<RealT>(it->second);
          }
        }
        coefficients_valid_ = true;
      }

      template <class ModelT>
      void evaluateAffine(const ModelT&                model,
                          JacobianMatrix<RealT, IdxT>& matrix,
                          ScalarT                      time,
                          RealT                        alpha)
      {
        if (!coefficients_valid_
            || coefficient_update_ == ComponentJacobianCoefficientUpdate::PerEvaluation)
        {
          refreshAffineCoefficients(model, time);
        }

        for (const auto& source : sources_)
        {
          const RealT scale = source.derivative ? alpha : RealT{1.0};
          for (const auto& entry : source.entries)
          {
            const RealT value = scale * entry.coefficient;
            if (value != RealT{0.0})
            {
              matrix.addToSlot(entry.slot, value);
            }
          }
        }
      }

      template <class ModelT>
      void evaluateGeneral(const ModelT&                model,
                           JacobianMatrix<RealT, IdxT>& matrix,
                           ScalarT                      time,
                           RealT                        alpha)
      {
        for (const auto& source : sources_)
        {
          if (source.entries.empty())
          {
            continue;
          }

          const auto source_index = static_cast<size_t>(source.active);
          seed_[source_index]     = ScalarT{1.0};
          std::fill(residual_.begin(), residual_.end(), ScalarT{0.0});
          std::fill(residual_dot_.begin(), residual_dot_.end(), ScalarT{0.0});

          GridKit::Enzyme::Sparse::__enzyme_fwddiff<void>(
              (void*) Detail::VectorResidualKernel<ModelT, ScalarT, IdxT>::eval,
              enzyme_const,
              &model,
              enzyme_dup,
              local_.data(),
              seed_.data(),
              enzyme_dup,
              residual_.data(),
              residual_dot_.data(),
              enzyme_const,
              time,
              enzyme_const,
              component_.variable_count,
              enzyme_const,
              component_.equation_count,
              enzyme_const,
              component_.electrical_port_count);

          seed_[source_index] = ScalarT{0.0};

          const RealT scale = source.derivative ? alpha : RealT{1.0};
          for (const auto& entry : source.entries)
          {
            const RealT value =
                scale * static_cast<RealT>(residual_dot_[static_cast<size_t>(entry.local_row)]);
            if (value != RealT{0.0})
            {
              matrix.addToSlot(entry.slot, value);
            }
          }
        }
      }

      ComponentLayout<IdxT> component_;
      IdxT                  residual_count_{0};
      IdxT                  port_size_{0};
      IdxT                  active_count_{0};

      std::vector<IdxT>    port_buses_;
      std::vector<IdxT>    input_port_variables_;
      std::vector<IdxT>    row_indices_;
      std::vector<Source>  sources_;
      std::vector<Nonzero> nonzeros_;

      std::vector<ScalarT>               local_;
      std::vector<ScalarT>               seed_;
      std::vector<ScalarT>               residual_;
      std::vector<ScalarT>               residual_dot_;
      bool                               coefficients_valid_{false};
      ComponentJacobianCoefficientUpdate coefficient_update_{
          ComponentJacobianCoefficientUpdate::PerEvaluation};
    };

    template <class ScalarT, typename IdxT>
    class JacobianPlan
    {
    public:
      using RealT  = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using Matrix = JacobianMatrix<RealT, IdxT>;
      using Entry  = typename Matrix::Entry;

    private:
      class BusResidualJacobianPlan
      {
      public:
        template <class DataT>
        void configure(const DataT& data, const Layout<IdxT>& layout, std::vector<Entry>& entries)
        {
          slots_.clear();
          for (IdxT bus = 0; bus < static_cast<IdxT>(data.buses.size()); ++bus)
          {
            if (!data.buses[static_cast<size_t>(bus)].hasFaultState())
            {
              continue;
            }

            for (IdxT phase = 0; phase < Layout<IdxT>::phases; ++phase)
            {
              entries.emplace_back(layout.busEquation(bus, phase), layout.busVariable(bus, phase));
              slots_.push_back({bus, phase, INVALID_INDEX<IdxT>});
            }
          }
        }

        void bindSlots(const Matrix& matrix, const Layout<IdxT>& layout)
        {
          for (auto& slot : slots_)
          {
            slot.slot = matrix.slot(layout.busEquation(slot.bus, slot.phase),
                                    layout.busVariable(slot.bus, slot.phase));
            if (slot.slot == INVALID_INDEX<IdxT>)
            {
              throw std::logic_error("EMT bus residual Jacobian slot map is missing a structural entry");
            }
          }
        }

        template <class DataT>
        void evaluate(const DataT& data, Matrix& matrix) const
        {
          for (const auto& slot : slots_)
          {
            const RealT value =
                data.buses[static_cast<size_t>(slot.bus)].residualJacobian(static_cast<size_t>(slot.phase));
            matrix.addToSlot(slot.slot, value);
          }
        }

      private:
        struct Slot
        {
          IdxT bus{INVALID_INDEX<IdxT>};
          IdxT phase{INVALID_INDEX<IdxT>};
          IdxT slot{INVALID_INDEX<IdxT>};
        };

        std::vector<Slot> slots_;
      };

    public:
      template <class DataT>
      void build(const DataT&                data,
                 const Layout<IdxT>&         layout,
                 const std::vector<ScalarT>& scratch_y,
                 const std::vector<ScalarT>& scratch_yp,
                 ScalarT                     scratch_time)
      {
        components_.clear();
        std::vector<Entry> entries;

        data.components.forEach(
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

        buses_.configure(data, layout, entries);
        matrix_.build(layout.size(), std::move(entries));
        buses_.bindSlots(matrix_, layout);
        for (auto& component : components_)
        {
          component.bindSlots(matrix_);
        }
      }

      template <class DataT>
      void evaluate(DataT&                      data,
                    const std::vector<ScalarT>& y,
                    const std::vector<ScalarT>& yp,
                    ScalarT                     time,
                    RealT                       alpha)
      {
        matrix_.clearValues();
        size_t plan_index = 0;
        data.components.forEach(
            [&](auto& component, ComponentId)
            {
              components_.at(plan_index).evaluate(component, matrix_, y, yp, time, alpha);
              ++plan_index;
            });
        buses_.evaluate(data, matrix_);
        matrix_.markUpdated();
      }

      void invalidateCachedCoefficients()
      {
        for (auto& component : components_)
        {
          component.invalidateCachedCoefficients();
        }
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
      BusResidualJacobianPlan                           buses_;
    };
  } // namespace EMT
} // namespace GridKit
