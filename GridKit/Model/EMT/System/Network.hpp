#pragma once

#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/System/Monitor.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    struct ComponentId
    {
      size_t type{INVALID_INDEX<size_t>};
      size_t index{INVALID_INDEX<size_t>};
    };

    constexpr bool operator==(const ComponentId& lhs, const ComponentId& rhs)
    {
      return lhs.type == rhs.type && lhs.index == rhs.index;
    }

    struct TerminalRef;
    struct InputRef;
    struct OutputRef;

    struct ComponentRef
    {
      ComponentId id;

      constexpr operator ComponentId() const
      {
        return id;
      }

      constexpr TerminalRef terminal(size_t local) const;
      constexpr InputRef    input(size_t local) const;
      constexpr OutputRef   output(size_t local) const;
    };

    struct TerminalRef
    {
      ComponentId component;
      size_t      index{INVALID_INDEX<size_t>};
    };

    template <typename IdxT>
    struct TerminalConnection
    {
      TerminalRef terminal;
      IdxT        bus{INVALID_INDEX<IdxT>};
    };

    struct OutputSpec
    {
      size_t variable{INVALID_INDEX<size_t>};
    };

    struct InputRef
    {
      ComponentId component;
      size_t      index{INVALID_INDEX<size_t>};
    };

    struct OutputRef
    {
      ComponentId component;
      size_t      index{INVALID_INDEX<size_t>};
    };

    struct PortConnection
    {
      OutputRef output;
      InputRef  input;
    };

    constexpr TerminalRef ComponentRef::terminal(size_t local) const
    {
      return {id, local};
    }

    constexpr InputRef ComponentRef::input(size_t local) const
    {
      return {id, local};
    }

    constexpr OutputRef ComponentRef::output(size_t local) const
    {
      return {id, local};
    }

    template <class ComponentT>
    struct TypedComponentRef
    {
      using component_type = ComponentT;

      ComponentId id;

      constexpr operator ComponentId() const
      {
        return id;
      }

      constexpr operator ComponentRef() const
      {
        return {id};
      }

      constexpr TerminalRef terminal(size_t local) const
      {
        return {id, local};
      }

      constexpr InputRef input(size_t local) const
      {
        return {id, local};
      }

      constexpr OutputRef output(size_t local) const
      {
        return {id, local};
      }
    };

    struct ComponentMonitorRequest
    {
      ComponentRef        component;
      std::string         label;
      std::vector<size_t> variables;
    };

    namespace Detail
    {
      template <typename T, typename... Ts>
      struct TypeIndex;

      template <typename T, typename... Ts>
      struct TypeIndex<T, T, Ts...> : std::integral_constant<size_t, 0>
      {
      };

      template <typename T, typename U, typename... Ts>
      struct TypeIndex<T, U, Ts...>
        : std::integral_constant<size_t, 1 + TypeIndex<T, Ts...>::value>
      {
      };

      template <typename T, typename... Ts>
      inline constexpr bool Contains = (std::is_same_v<T, Ts> || ...);

      template <class T>
      inline constexpr bool AlwaysFalse = false;

      template <class T>
      consteval bool hasDifferential()
      {
        if constexpr (T::variable_count == 0)
        {
          return true;
        }
        else
        {
          return requires(size_t local) {
            { T::differential(local) } -> std::convertible_to<bool>;
          };
        }
      }
    } // namespace Detail

    template <class T>
    struct ComponentTraits
    {
      static constexpr size_t variable_count = T::variable_count;
      static constexpr size_t equation_count = T::equation_count;
      static constexpr size_t terminal_count = T::terminal_count;
      static constexpr size_t input_count    = T::input_count;
      static constexpr size_t output_count   = T::output_count;

      static constexpr bool has_differential = Detail::hasDifferential<T>();
      static constexpr bool is_valid         = variable_count == 0 || has_differential;

      static constexpr bool differential(size_t local)
      {
        if constexpr (variable_count == 0)
        {
          (void) local;
          return false;
        }
        else if constexpr (has_differential)
        {
          return T::differential(local);
        }
        else
        {
          static_assert(Detail::AlwaysFalse<T>,
                        "EMT components with local variables must define static constexpr bool differential(size_t)");
        }
      }

      static constexpr OutputSpec output(size_t index)
      {
        if constexpr (output_count == 0)
        {
          (void) index;
          return {};
        }
        else
        {
          return T::output(index);
        }
      }
    };

    template <class... Ts>
    class ComponentStore
    {
    public:
      template <class T>
      auto add(T component)
      {
        using ComponentT = std::decay_t<T>;
        static_assert(Detail::Contains<ComponentT, Ts...>,
                      "Component type is not part of this EMT network");
        static_assert(ComponentTraits<ComponentT>::is_valid,
                      "EMT components with local variables must define static constexpr bool differential(size_t)");

        auto&                         components = std::get<std::vector<ComponentT>>(components_);
        TypedComponentRef<ComponentT> ref{{Detail::TypeIndex<ComponentT, Ts...>::value, components.size()}};
        components.push_back(std::move(component));
        return ref;
      }

      template <class T>
      std::vector<T>& get()
      {
        return std::get<std::vector<T>>(components_);
      }

      template <class T>
      const std::vector<T>& get() const
      {
        return std::get<std::vector<T>>(components_);
      }

      template <class Fn>
      void forEach(Fn&& fn)
      {
        std::apply(
            [&](auto&... vectors)
            {
              (forEachVector(vectors, fn), ...);
            },
            components_);
      }

      template <class Fn>
      void forEach(Fn&& fn) const
      {
        std::apply(
            [&](const auto&... vectors)
            {
              (forEachVector(vectors, fn), ...);
            },
            components_);
      }

      template <class Fn>
      bool visit(ComponentId id, Fn&& fn)
      {
        bool found = false;
        forEach(
            [&](auto& component, ComponentId current)
            {
              if (!found && current == id)
              {
                fn(component);
                found = true;
              }
            });
        return found;
      }

      template <class Fn>
      bool visit(ComponentId id, Fn&& fn) const
      {
        bool found = false;
        forEach(
            [&](const auto& component, ComponentId current)
            {
              if (!found && current == id)
              {
                fn(component);
                found = true;
              }
            });
        return found;
      }

      static constexpr size_t typeCount()
      {
        return sizeof...(Ts);
      }

    private:
      template <class VectorT, class Fn>
      static void forEachVector(VectorT& components, Fn& fn)
      {
        using ComponentT      = typename std::remove_cvref_t<VectorT>::value_type;
        constexpr size_t type = Detail::TypeIndex<ComponentT, Ts...>::value;
        for (size_t i = 0; i < components.size(); ++i)
        {
          fn(components[i], ComponentId{type, i});
        }
      }

      std::tuple<std::vector<Ts>...> components_;
    };

    template <class RealT, typename IdxT, class... ComponentTs>
    struct NetworkData
    {
      using scalar_type          = RealT;
      using index_type           = IdxT;
      using component_store_type = ComponentStore<ComponentTs...>;
      using MonitorSinkSpec      = GridKit::Model::VariableMonitorBase::SinkSpec;

      std::vector<Bus<RealT, IdxT>>         buses;
      component_store_type                  components;
      std::vector<TerminalConnection<IdxT>> terminal_connections;
      std::vector<PortConnection>           port_connections;
      std::vector<MonitorSinkSpec>          monitor_sinks;
      std::vector<BusMonitorRequest<IdxT>>  bus_monitors;
      std::vector<ComponentMonitorRequest>  component_monitors;

      IdxT addBus(BusData<RealT, IdxT> data)
      {
        const IdxT id = static_cast<IdxT>(buses.size());
        buses.emplace_back(data);
        return id;
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

      void connect(OutputRef output, InputRef input)
      {
        port_connections.push_back({output, input});
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
    };
  } // namespace EMT
} // namespace GridKit
