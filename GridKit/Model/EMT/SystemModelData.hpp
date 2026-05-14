#pragma once

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/System/Components.hpp>
#include <GridKit/Model/EMT/System/Events.hpp>
#include <GridKit/Model/EMT/System/Monitor.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT, class... ComponentTs>
    struct SystemModelData
    {
      using scalar_type          = RealT;
      using index_type           = IdxT;
      using component_types      = std::tuple<ComponentTs...>;
      using component_store_type = ComponentStore<ComponentTs...>;
      using MonitorSinkSpec      = GridKit::Model::VariableMonitorBase::SinkSpec;
      using self_type            = SystemModelData<RealT, IdxT, ComponentTs...>;

      struct ScheduledEvent
      {
        double                                time{0.0};
        size_t                                order{0};
        std::function<void(const self_type&)> validate;
        std::function<void(self_type&)>       prepare;
        std::function<void(self_type&)>       apply;
      };

      std::vector<Bus<RealT, IdxT>>         buses;
      component_store_type                  components;
      std::vector<TerminalConnection<IdxT>> terminal_connections;
      std::vector<SignalConnection>         signal_connections;
      std::vector<MonitorSinkSpec>          monitor_sinks;
      std::vector<BusMonitorRequest<IdxT>>  bus_monitors;
      std::vector<ComponentMonitorRequest>  component_monitors;
      std::vector<ScheduledEvent>           events;

      IdxT addBus(BusData<RealT, IdxT> data)
      {
        const IdxT id = static_cast<IdxT>(buses.size());
        buses.emplace_back(data);
        return id;
      }

      BusRef<IdxT> busRef(IdxT bus) const
      {
        return {bus};
      }

      template <class T>
      auto add(T component)
      {
        return components.add(std::move(component));
      }

      void connect(TerminalRef terminal, IdxT bus)
      {
        terminal_connections.push_back({terminal, bus});
      }

      void connect(SignalOutputRef output, SignalInputRef input)
      {
        signal_connections.push_back({output, input});
      }

      void addMonitorSink(MonitorSinkSpec sink)
      {
        monitor_sinks.push_back(std::move(sink));
      }

      void monitorBus(IdxT                                      bus,
                      std::string                               label,
                      std::initializer_list<BusMonitorVariable> variables)
      {
        bus_monitors.push_back({bus, std::move(label), {variables.begin(), variables.end()}});
      }

      template <class ComponentT>
      void monitorComponent(TypedComponentRef<ComponentT>                                                component,
                            std::string                                                                  label,
                            std::initializer_list<typename ComponentMonitorTraits<ComponentT>::Variable> variables)
      {
        std::vector<size_t> encoded;
        encoded.reserve(variables.size());
        for (auto variable : variables)
        {
          encoded.push_back(static_cast<size_t>(variable));
        }
        component_monitors.push_back({ComponentRef{component.id}, std::move(label), std::move(encoded)});
      }

      template <class ActionT>
      void schedule(double time, BusRef<IdxT> bus, ActionT action)
      {
        const size_t order = events.size();
        events.push_back({time,
                          order,
                          [bus](const self_type& data)
                          {
                            if (bus.id >= static_cast<IdxT>(data.buses.size()))
                            {
                              throw std::invalid_argument("EMT event target bus does not exist");
                            }
                            using BusT = Bus<RealT, IdxT>;
                            if constexpr (!EventTraits<BusT>::template supports<ActionT>())
                            {
                              throw std::invalid_argument("EMT bus does not support scheduled event action");
                            }
                          },
                          [bus, action](self_type& data)
                          {
                            (void) bus;
                            (void) action;
                            using BusT = Bus<RealT, IdxT>;
                            if constexpr (EventTraits<BusT>::template supports<ActionT>())
                            {
                              EventTraits<BusT>::prepare(data.buses[static_cast<size_t>(bus.id)], action);
                            }
                          },
                          [bus, action](self_type& data)
                          {
                            (void) bus;
                            (void) action;
                            using BusT = Bus<RealT, IdxT>;
                            if constexpr (!EventTraits<BusT>::template supports<ActionT>())
                            {
                              throw std::invalid_argument("EMT bus does not support scheduled event action");
                            }
                            else
                            {
                              EventTraits<BusT>::apply(data.buses[static_cast<size_t>(bus.id)], action);
                            }
                          }});
      }

      template <class ActionT>
      void schedule(double time, ComponentRef component, ActionT action)
      {
        const size_t order = events.size();
        events.push_back({time,
                          order,
                          [component](const self_type& data)
                          {
                            const bool found = data.components.visit(
                                component.id,
                                [&](const auto& stored_component)
                                {
                                  using StoredT = std::decay_t<decltype(stored_component)>;
                                  (void) stored_component;
                                  if constexpr (!EventTraits<StoredT>::template supports<ActionT>())
                                  {
                                    throw std::invalid_argument("EMT component does not support scheduled event action");
                                  }
                                });
                            if (!found)
                            {
                              throw std::invalid_argument("EMT event target component does not exist");
                            }
                          },
                          [component, action](self_type& data)
                          {
                            (void) action;
                            (void) data.components.visit(
                                component.id,
                                [&](auto& stored_component)
                                {
                                  using StoredT = std::decay_t<decltype(stored_component)>;
                                  if constexpr (EventTraits<StoredT>::template supports<ActionT>())
                                  {
                                    EventTraits<StoredT>::prepare(stored_component, action);
                                  }
                                });
                          },
                          [component, action](self_type& data)
                          {
                            (void) action;
                            const bool found = data.components.visit(
                                component.id,
                                [&](auto& stored_component)
                                {
                                  using StoredT = std::decay_t<decltype(stored_component)>;
                                  if constexpr (!EventTraits<StoredT>::template supports<ActionT>())
                                  {
                                    throw std::invalid_argument("EMT component does not support scheduled event action");
                                  }
                                  else
                                  {
                                    EventTraits<StoredT>::apply(stored_component, action);
                                  }
                                });
                            if (!found)
                            {
                              throw std::invalid_argument("EMT event target component does not exist");
                            }
                          }});
      }

      template <class ComponentT, class ActionT>
      void schedule(double time, TypedComponentRef<ComponentT> component, ActionT action)
      {
        schedule(time, ComponentRef{component.id}, action);
      }
    };
  } // namespace EMT
} // namespace GridKit
