#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/CallbackVariableMonitor.hpp>
#include <GridKit/Model/EMT/System/Events.hpp>
#include <GridKit/Model/EMT/System/Jacobian.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>
#include <GridKit/Model/EMT/System/Views.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/Model/VariableMonitorController.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class DataT>
    class SystemModel
      : public GridKit::Model::Evaluator<typename DataT::scalar_type, typename DataT::index_type>
    {
    public:
      using ScalarT    = typename DataT::scalar_type;
      using IdxT       = typename DataT::index_type;
      using StoreT     = typename DataT::component_store_type;
      using EventT     = typename DataT::ScheduledEvent;
      using Base       = GridKit::Model::Evaluator<ScalarT, IdxT>;
      using RealT      = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using MatrixT    = typename Base::MatrixT;
      using CsrMatrixT = typename Base::CsrMatrixT;

      explicit SystemModel(DataT data,
                           RealT rel_tol   = 1e-4,
                           RealT abs_tol   = 1e-4,
                           bool  use_jac   = true,
                           IdxT  max_steps = 2000)
        : data_{std::move(data)},
          rel_tol_{rel_tol},
          abs_tol_{abs_tol},
          use_jac_{use_jac},
          max_steps_{max_steps}
      {
      }

      int allocate() override
      {
        allocated_ = false;
        buildLayout();
        if (layout_.size() != layout_.equationCount())
        {
          throw std::invalid_argument("EMT layout must have one residual equation per variable");
        }

        y_.assign(static_cast<size_t>(layout_.size()), ScalarT{0.0});
        yp_.assign(static_cast<size_t>(layout_.size()), ScalarT{0.0});
        f_.assign(static_cast<size_t>(layout_.size()), ScalarT{0.0});
        tag_.assign(static_cast<size_t>(layout_.size()), false);
        differential_variables_.assign(static_cast<size_t>(layout_.size()), false);

        markDifferentialVariables();
        initializeEvents();
        initializeMonitor();
        buildJacobianPlan();
        allocated_ = true;
        return 0;
      }

      int initialize() override
      {
        ensureAllocated();
        std::fill(y_.begin(), y_.end(), ScalarT{0.0});
        std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
        std::fill(f_.begin(), f_.end(), ScalarT{0.0});

        for (IdxT bus = 0; bus < static_cast<IdxT>(data_.buses.size()); ++bus)
        {
          data_.buses[static_cast<size_t>(bus)].initialize(y_.data(), yp_.data(), layout_.busVariable(bus, 0));
        }

        data_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              const auto&                     slot = layout_.component(id);
              InitialStateView<ScalarT, IdxT> init(y_.data(),
                                                   yp_.data(),
                                                   layout_,
                                                   slot,
                                                   layout_.terminalBuses(id),
                                                   std::span<const Bus<ScalarT, IdxT>>(data_.buses.data(),
                                                                                       data_.buses.size()));
              if constexpr (requires(InitialStateView<ScalarT, IdxT>& view) { component.initialize(view); })
              {
                component.initialize(init);
              }
            });
        resetEventCursor();
        return 0;
      }

      int tagDifferentiable() override
      {
        ensureAllocated();
        tag_ = differential_variables_;
        return 0;
      }

      int evaluateResidual() override
      {
        ensureAllocated();
        std::fill(f_.begin(), f_.end(), ScalarT{0.0});
        data_.components.forEach(
            [&](auto& component, ComponentId id)
            {
              const auto&                 slot = layout_.component(id);
              StateView<ScalarT, IdxT>    state(y_.data(),
                                             yp_.data(),
                                             layout_,
                                             slot,
                                             layout_.terminalBuses(id),
                                             layout_.inputVariables(id),
                                             static_cast<ScalarT>(time_));
              EquationView<ScalarT, IdxT> equations(f_.data(), layout_, slot, layout_.terminalBuses(id));
              component.residual(state, equations);
            });
        evaluateBusResiduals();
        return 0;
      }

      int evaluateJacobian() override
      {
        ensureAllocated();
        jacobian_.evaluate(data_, y_, yp_, static_cast<ScalarT>(time_), alpha_);
        return 0;
      }

      int evaluateIntegrand() override
      {
        return 0;
      }

      int initializeAdjoint() override
      {
        return 0;
      }

      int evaluateAdjointResidual() override
      {
        return 0;
      }

      int evaluateAdjointIntegrand() override
      {
        return 0;
      }

      IdxT size() override
      {
        return layout_.size();
      }

      IdxT nnz() override
      {
        return jacobian_.matrix().nnz();
      }

      CsrMatrixT* getCsrJacobian() const override
      {
        return jacobian_.matrix().matrix();
      }

      bool monitoring() const override
      {
        return monitor_active_;
      }

      void printMonitoredVariables() const override
      {
        if (monitor_active_)
        {
          monitor_.print();
        }
      }

      const GridKit::Model::VariableMonitorBase* getMonitor() const override
      {
        return &monitor_;
      }

      void startMonitor() override
      {
        if (!monitor_.empty())
        {
          monitor_.start();
          monitor_active_ = true;
        }
      }

      void stopMonitor() override
      {
        if (monitor_active_)
        {
          monitor_.stop();
          monitor_active_ = false;
        }
      }

      bool hasJacobian() override
      {
        return use_jac_;
      }

      IdxT sizeQuadrature() override
      {
        return static_cast<IdxT>(integrand_.size());
      }

      IdxT sizeParams() override
      {
        return static_cast<IdxT>(param_.size());
      }

      void updateTime(RealT t, RealT a) override
      {
        time_  = t;
        alpha_ = a;
      }

      void setTolerances(RealT& rtol, RealT& atol) const override
      {
        rtol = rel_tol_;
        atol = abs_tol_;
      }

      void setMaxSteps(IdxT& msa) const override
      {
        msa = max_steps_;
      }

      std::vector<ScalarT>& y() override
      {
        return y_;
      }

      const std::vector<ScalarT>& y() const override
      {
        return y_;
      }

      std::vector<ScalarT>& yp() override
      {
        return yp_;
      }

      const std::vector<ScalarT>& yp() const override
      {
        return yp_;
      }

      std::vector<bool>& tag() override
      {
        return tag_;
      }

      const std::vector<bool>& tag() const override
      {
        return tag_;
      }

      std::vector<ScalarT>& yB() override
      {
        return yB_;
      }

      const std::vector<ScalarT>& yB() const override
      {
        return yB_;
      }

      std::vector<ScalarT>& ypB() override
      {
        return ypB_;
      }

      const std::vector<ScalarT>& ypB() const override
      {
        return ypB_;
      }

      std::vector<ScalarT>& param() override
      {
        return param_;
      }

      const std::vector<ScalarT>& param() const override
      {
        return param_;
      }

      std::vector<ScalarT>& param_up() override
      {
        return param_up_;
      }

      const std::vector<ScalarT>& param_up() const override
      {
        return param_up_;
      }

      std::vector<ScalarT>& param_lo() override
      {
        return param_lo_;
      }

      const std::vector<ScalarT>& param_lo() const override
      {
        return param_lo_;
      }

      std::vector<ScalarT>& getResidual() override
      {
        return f_;
      }

      const std::vector<ScalarT>& getResidual() const override
      {
        return f_;
      }

      MatrixT& getJacobian() override
      {
        return coo_jacobian_;
      }

      const MatrixT& getJacobian() const override
      {
        return coo_jacobian_;
      }

      std::vector<ScalarT>& getIntegrand() override
      {
        return integrand_;
      }

      const std::vector<ScalarT>& getIntegrand() const override
      {
        return integrand_;
      }

      std::vector<ScalarT>& getAdjointResidual() override
      {
        return adjoint_residual_;
      }

      const std::vector<ScalarT>& getAdjointResidual() const override
      {
        return adjoint_residual_;
      }

      std::vector<ScalarT>& getAdjointIntegrand() override
      {
        return adjoint_integrand_;
      }

      const std::vector<ScalarT>& getAdjointIntegrand() const override
      {
        return adjoint_integrand_;
      }

      const Layout<IdxT>& layout() const
      {
        return layout_;
      }

      const JacobianMatrix<RealT, IdxT>& assembly() const
      {
        return jacobian_.matrix();
      }

      DataT& data()
      {
        return data_;
      }

      const DataT& data() const
      {
        return data_;
      }

      std::optional<RealT> nextEventTime() const
      {
        if (event_cursor_ >= event_schedule_.size())
        {
          return std::nullopt;
        }
        return static_cast<RealT>(event_schedule_[event_cursor_].time);
      }

      bool applyNextEventBatch()
      {
        ensureAllocated();
        if (event_cursor_ >= event_schedule_.size())
        {
          return false;
        }

        const double event_time = event_schedule_[event_cursor_].time;
        while (event_cursor_ < event_schedule_.size()
               && event_schedule_[event_cursor_].time == event_time)
        {
          applyEvent(event_schedule_[event_cursor_]);
          ++event_cursor_;
        }
        return true;
      }

      void resetEventCursor()
      {
        event_cursor_ = 0;
      }

    private:
      class BusMonitorBindingContext
      {
      public:
        using scalar_type = ScalarT;
        using real_type   = RealT;
        using index_type  = IdxT;
        using MonitorT    = GridKit::Model::CallbackVariableMonitor<ScalarT>;

        BusMonitorBindingContext(SystemModel& system, IdxT bus, MonitorT& monitor)
          : system_(&system),
            bus_(bus),
            monitor_(monitor)
        {
        }

        template <class FuncT>
        void add(std::string label, FuncT getter)
        {
          monitor_.add(std::move(label), std::move(getter));
        }

        void addVoltage(std::string label, IdxT phase)
        {
          addVectorValue(std::move(label), false, phase);
        }

        void addVoltageDerivative(std::string label, IdxT phase)
        {
          addVectorValue(std::move(label), true, phase);
        }

        IdxT voltageIndex(IdxT phase) const
        {
          validatePhase(phase);
          return system_->layout_.busVariable(bus_, phase);
        }

        const std::vector<ScalarT>* y() const
        {
          return &system_->y_;
        }

      private:
        void addVectorValue(std::string label, bool derivative, IdxT phase)
        {
          const IdxT index = voltageIndex(phase);
          monitor_.add(std::move(label),
                       [system = system_, index, derivative]()
                       {
                         const auto& values = derivative ? system->yp_ : system->y_;
                         return values[static_cast<size_t>(index)];
                       });
        }

        static void validatePhase(IdxT phase)
        {
          if (phase >= Layout<IdxT>::phases)
          {
            throw std::invalid_argument("EMT monitor phase index is out of range");
          }
        }

        SystemModel* system_{nullptr};
        IdxT         bus_{INVALID_INDEX<IdxT>};
        MonitorT&    monitor_;
      };

      class MonitorBindingContext
      {
      public:
        using scalar_type = ScalarT;
        using real_type   = RealT;
        using index_type  = IdxT;
        using MonitorT    = GridKit::Model::CallbackVariableMonitor<ScalarT>;

        MonitorBindingContext(SystemModel&                 system,
                              const ComponentLayout<IdxT>& component,
                              std::span<const IdxT>        terminal_buses,
                              MonitorT&                    monitor)
          : system_(&system),
            component_(component),
            terminal_buses_(terminal_buses),
            monitor_(monitor)
        {
        }

        template <class FuncT>
        void add(std::string label, FuncT getter)
        {
          monitor_.add(std::move(label), std::move(getter));
        }

        void addState(std::string label, size_t local)
        {
          addVectorValue(std::move(label), false, local);
        }

        void addDerivative(std::string label, size_t local)
        {
          addVectorValue(std::move(label), true, local);
        }

        IdxT terminalVoltageIndex(size_t terminal, IdxT phase) const
        {
          if (terminal >= terminal_buses_.size())
          {
            throw std::invalid_argument("EMT monitor terminal index is out of range");
          }
          if (phase >= Layout<IdxT>::phases)
          {
            throw std::invalid_argument("EMT monitor phase index is out of range");
          }
          return system_->layout_.busVariable(terminal_buses_[terminal], phase);
        }

        const std::vector<ScalarT>* y() const
        {
          return &system_->y_;
        }

        const RealT* time() const
        {
          return &system_->time_;
        }

      private:
        void addVectorValue(std::string label, bool derivative, size_t local)
        {
          if (local >= static_cast<size_t>(component_.variable_count))
          {
            throw std::invalid_argument("EMT monitor component variable index is out of range");
          }

          const IdxT index = component_.variable_offset + static_cast<IdxT>(local);
          monitor_.add(std::move(label),
                       [system = system_, index, derivative]()
                       {
                         const auto& values = derivative ? system->yp_ : system->y_;
                         return values[static_cast<size_t>(index)];
                       });
        }

        SystemModel*          system_{nullptr};
        ComponentLayout<IdxT> component_{};
        std::span<const IdxT> terminal_buses_;
        MonitorT&             monitor_;
      };

      class BusResidualContext
      {
      public:
        BusResidualContext(SystemModel& system, IdxT bus)
          : system_(&system),
            bus_(bus)
        {
        }

        size_t phaseCount() const
        {
          return static_cast<size_t>(Layout<IdxT>::phases);
        }

        ScalarT voltage(IdxT phase) const
        {
          return system_->y_[static_cast<size_t>(system_->layout_.busVariable(bus_, phase))];
        }

        void addCurrent(IdxT phase, ScalarT value)
        {
          system_->f_[static_cast<size_t>(system_->layout_.busEquation(bus_, phase))] += value;
        }

      private:
        SystemModel* system_{nullptr};
        IdxT         bus_{INVALID_INDEX<IdxT>};
      };

      void ensureAllocated()
      {
        if (!allocated_)
        {
          allocate();
        }
      }

      void buildLayout()
      {
        layout_.reset(static_cast<IdxT>(data_.buses.size()), StoreT::typeCount());
        data_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              static_assert(ComponentTraits<ComponentT>::is_valid,
                            "EMT components with local variables must define static constexpr bool differential(size_t)");
              layout_.appendComponent(id.type,
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::variable_count),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::equation_count),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::terminal_count),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::input_count));
            });

        layout_.allocateConnections();
        bindTerminalConnections();
        bindSignalConnections();
        layout_.validateConnections();
      }

      void initializeMonitor()
      {
        monitor_.clear();
        monitor_objects_.clear();
        monitor_active_ = false;

        for (const auto& sink : data_.monitor_sinks)
        {
          monitor_.addSink(sink);
        }

        bindBusMonitors();
        bindComponentMonitors();

        if (!data_.monitor_sinks.empty())
        {
          startMonitor();
        }
      }

      void initializeEvents()
      {
        event_schedule_ = data_.events;
        for (const auto& event : event_schedule_)
        {
          validateEventTime(event);
        }

        std::stable_sort(event_schedule_.begin(),
                         event_schedule_.end(),
                         [](const auto& lhs, const auto& rhs)
                         {
                           if (lhs.time == rhs.time)
                           {
                             return lhs.order < rhs.order;
                           }
                           return lhs.time < rhs.time;
                         });
        event_cursor_ = 0;

        for (auto& event : event_schedule_)
        {
          if (!event.validate || !event.prepare || !event.apply)
          {
            throw std::invalid_argument("EMT scheduled event is incomplete");
          }
          event.validate(data_);
          event.prepare(data_);
        }
      }

      void validateEventTime(const EventT& event)
      {
        if (!std::isfinite(event.time) || event.time < 0.0)
        {
          throw std::invalid_argument("EMT event time must be finite and nonnegative");
        }
      }

      void applyEvent(const EventT& event)
      {
        if (!event.apply)
        {
          throw std::invalid_argument("EMT scheduled event is incomplete");
        }
        event.apply(data_);
      }

      void bindBusMonitors()
      {
        for (const auto& request : data_.bus_monitors)
        {
          if (request.bus >= layout_.busCount())
          {
            throw std::invalid_argument("EMT monitor bus index is out of range");
          }

          auto                     monitor = std::make_unique<GridKit::Model::CallbackVariableMonitor<ScalarT>>(request.label);
          BusMonitorBindingContext context(*this, request.bus, *monitor);
          const auto&              bus = data_.buses[static_cast<size_t>(request.bus)];
          for (auto variable : request.variables)
          {
            BusMonitorTraits<Bus<ScalarT, IdxT>>::bind(context, bus, variable);
          }

          auto* raw = monitor.get();
          monitor_objects_.push_back(std::move(monitor));
          monitor_.addMonitor(raw);
        }
      }

      void bindComponentMonitors()
      {
        for (const auto& request : data_.component_monitors)
        {
          const bool found = data_.components.visit(
              request.component,
              [&](const auto& component)
              {
                using ComponentT = std::decay_t<decltype(component)>;

                if constexpr (requires { typename ComponentMonitorTraits<ComponentT>::Variable; })
                {
                  auto                  monitor = std::make_unique<GridKit::Model::CallbackVariableMonitor<ScalarT>>(request.label);
                  MonitorBindingContext context(*this,
                                                layout_.component(request.component),
                                                layout_.terminalBuses(request.component),
                                                *monitor);

                  for (auto raw_variable : request.variables)
                  {
                    ComponentMonitorTraits<ComponentT>::bind(context, component, raw_variable);
                  }

                  auto* raw = monitor.get();
                  monitor_objects_.push_back(std::move(monitor));
                  monitor_.addMonitor(raw);
                }
                else
                {
                  throw std::invalid_argument("EMT component type does not define monitor variables");
                }
              });

          if (!found)
          {
            throw std::invalid_argument("EMT monitor component does not exist");
          }
        }
      }

      void bindTerminalConnections()
      {
        for (const auto& connection : data_.terminal_connections)
        {
          if (connection.bus >= static_cast<IdxT>(data_.buses.size()))
          {
            throw std::invalid_argument("EMT terminal connection references a bus that does not exist");
          }

          const bool found = data_.components.visit(
              connection.terminal.component,
              [&](const auto& component)
              {
                using ComponentT = std::decay_t<decltype(component)>;
                (void) component;
                if (connection.terminal.index >= ComponentTraits<ComponentT>::terminal_count)
                {
                  throw std::invalid_argument("EMT terminal index is out of range");
                }
                layout_.connectTerminal(connection.terminal.component,
                                        connection.terminal.index,
                                        connection.bus);
              });

          if (!found)
          {
            throw std::invalid_argument("EMT terminal connection component does not exist");
          }
        }
      }

      void bindSignalConnections()
      {
        for (const auto& connection : data_.signal_connections)
        {
          layout_.connectInput(connection.input.component,
                               connection.input.index,
                               resolveSignalOutputVariable(connection.output));
        }
      }

      void evaluateBusResiduals()
      {
        using BusT = Bus<ScalarT, IdxT>;
        for (IdxT bus = 0; bus < static_cast<IdxT>(data_.buses.size()); ++bus)
        {
          BusResidualContext context(*this, bus);
          ResidualTraits<BusT>::evaluate(context, data_.buses[static_cast<size_t>(bus)]);
        }
      }

      IdxT resolveSignalOutputVariable(const SignalOutputRef& output) const
      {
        IdxT       variable = INVALID_INDEX<IdxT>;
        const bool found    = data_.components.visit(
            output.component,
            [&](const auto& component)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              if (output.index >= ComponentTraits<ComponentT>::output_count)
              {
                throw std::invalid_argument("EMT output index is out of range");
              }

              const SignalOutputSpec spec = ComponentTraits<ComponentT>::output(output.index);
              if (spec.variable >= ComponentTraits<ComponentT>::variable_count)
              {
                throw std::invalid_argument("EMT output variable is out of range");
              }
              variable = layout_.component(output.component).variable_offset + static_cast<IdxT>(spec.variable);
            });

        if (!found)
        {
          throw std::invalid_argument("EMT output component does not exist");
        }
        return variable;
      }

      void markDifferentialVariables()
      {
        data_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              const auto& slot = layout_.component(id);
              for (IdxT local = 0; local < slot.variable_count; ++local)
              {
                differential_variables_[static_cast<size_t>(slot.variable_offset + local)] =
                    ComponentTraits<ComponentT>::differential(static_cast<size_t>(local));
              }
            });
      }

      void buildJacobianPlan()
      {
        std::vector<ScalarT> scratch_y(static_cast<size_t>(layout_.size()), ScalarT{0.0});
        std::vector<ScalarT> scratch_yp(static_cast<size_t>(layout_.size()), ScalarT{0.0});
        initializeScratchState(scratch_y, scratch_yp);
        jacobian_.build(data_, layout_, scratch_y, scratch_yp, ScalarT{0.125});
      }

      static ScalarT deterministicScratchValue(IdxT index)
      {
        const IdxT cycle = index % IdxT{23};
        return ScalarT{0.125} + ScalarT{0.03125} * static_cast<ScalarT>(cycle + IdxT{1});
      }

      void initializeScratchState(std::vector<ScalarT>& y, std::vector<ScalarT>& yp) const
      {
        for (IdxT i = 0; i < static_cast<IdxT>(y.size()); ++i)
        {
          y[static_cast<size_t>(i)]  = deterministicScratchValue(i);
          yp[static_cast<size_t>(i)] = deterministicScratchValue(i + static_cast<IdxT>(y.size()));
        }
      }

      DataT                       data_;
      Layout<IdxT>                layout_;
      JacobianPlan<ScalarT, IdxT> jacobian_;

      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<ScalarT> f_;
      std::vector<bool>    tag_;
      std::vector<bool>    differential_variables_;

      std::vector<ScalarT> yB_;
      std::vector<ScalarT> ypB_;
      std::vector<ScalarT> param_;
      std::vector<ScalarT> param_up_;
      std::vector<ScalarT> param_lo_;
      std::vector<ScalarT> integrand_;
      std::vector<ScalarT> adjoint_residual_;
      std::vector<ScalarT> adjoint_integrand_;

      MatrixT coo_jacobian_;

      RealT                                                             rel_tol_{1e-4};
      RealT                                                             abs_tol_{1e-4};
      RealT                                                             time_{0.0};
      RealT                                                             alpha_{0.0};
      GridKit::Model::VariableMonitorController<ScalarT>                monitor_{time_};
      std::vector<std::unique_ptr<GridKit::Model::VariableMonitorBase>> monitor_objects_;
      std::vector<EventT>                                               event_schedule_;
      size_t                                                            event_cursor_{0};
      bool                                                              monitor_active_{false};
      bool                                                              use_jac_{true};
      bool                                                              allocated_{false};
      IdxT                                                              max_steps_{2000};
    };
  } // namespace EMT
} // namespace GridKit
