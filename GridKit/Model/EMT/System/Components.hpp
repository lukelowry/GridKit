#pragma once

#include <concepts>
#include <cstddef>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Constants.hpp>

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

    enum class PortKind
    {
      Electrical,
      Input,
      Output
    };

    template <PortKind Kind>
    struct PortRef
    {
      ComponentId component;
      size_t      index{INVALID_INDEX<size_t>};
    };

    using ElectricalPortRef = PortRef<PortKind::Electrical>;
    using InputPortRef      = PortRef<PortKind::Input>;
    using OutputPortRef     = PortRef<PortKind::Output>;

    struct ComponentRef
    {
      ComponentId id;

      constexpr operator ComponentId() const
      {
        return id;
      }

      constexpr ElectricalPortRef port(size_t local) const;
      constexpr InputPortRef      inputPort(size_t local) const;
      constexpr OutputPortRef     outputPort(size_t local) const;
    };

    template <typename IdxT>
    struct BusRef
    {
      IdxT id{INVALID_INDEX<IdxT>};
    };

    template <typename IdxT>
    struct PortConnection
    {
      ElectricalPortRef port;
      IdxT              bus{INVALID_INDEX<IdxT>};
    };

    struct OutputPortSpec
    {
      size_t variable{INVALID_INDEX<size_t>};
    };

    struct SignalPortConnection
    {
      OutputPortRef output;
      InputPortRef  input;
    };

    constexpr ElectricalPortRef ComponentRef::port(size_t local) const
    {
      return {id, local};
    }

    constexpr InputPortRef ComponentRef::inputPort(size_t local) const
    {
      return {id, local};
    }

    constexpr OutputPortRef ComponentRef::outputPort(size_t local) const
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

      constexpr ElectricalPortRef port(size_t local) const
      {
        return {id, local};
      }

      constexpr InputPortRef inputPort(size_t local) const
      {
        return {id, local};
      }

      constexpr OutputPortRef outputPort(size_t local) const
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
      consteval bool hasRuntimeVariableCount()
      {
        return requires(const T& component) {
          { component.variableCount() } -> std::convertible_to<size_t>;
        };
      }

      template <class T>
      consteval bool hasRuntimeEquationCount()
      {
        return requires(const T& component) {
          { component.equationCount() } -> std::convertible_to<size_t>;
        };
      }

      template <class T>
      consteval bool hasStaticDifferential()
      {
        return requires(size_t local) {
          { T::differential(local) } -> std::convertible_to<bool>;
        };
      }

      template <class T>
      consteval bool hasRuntimeDifferential()
      {
        return requires(const T& component, size_t local) {
          { component.differential(local) } -> std::convertible_to<bool>;
        };
      }

      template <class T>
      consteval bool hasDifferential()
      {
        if constexpr (T::variable_count == 0)
        {
          return true;
        }
        else
        {
          return hasStaticDifferential<T>() || hasRuntimeDifferential<T>();
        }
      }
    } // namespace Detail

    inline constexpr size_t dynamic_component_count = INVALID_INDEX<size_t>;

    template <class T>
    struct ComponentTraits
    {
      static constexpr size_t variable_count        = T::variable_count;
      static constexpr size_t equation_count        = T::equation_count;
      static constexpr size_t electrical_port_count = T::electrical_port_count;
      static constexpr size_t input_port_count      = T::input_port_count;
      static constexpr size_t output_port_count     = T::output_port_count;

      static constexpr bool has_dynamic_counts =
          variable_count == dynamic_component_count || equation_count == dynamic_component_count;
      static constexpr bool has_runtime_counts =
          Detail::hasRuntimeVariableCount<T>() && Detail::hasRuntimeEquationCount<T>();
      static constexpr bool has_differential = Detail::hasDifferential<T>();
      static constexpr bool is_valid         = (!has_dynamic_counts || has_runtime_counts)
                                       && (variable_count == 0 || has_differential);

      static size_t variableCount(const T& component)
      {
        if constexpr (variable_count == dynamic_component_count)
        {
          return component.variableCount();
        }
        else
        {
          (void) component;
          return variable_count;
        }
      }

      static size_t equationCount(const T& component)
      {
        if constexpr (equation_count == dynamic_component_count)
        {
          return component.equationCount();
        }
        else
        {
          (void) component;
          return equation_count;
        }
      }

      static constexpr bool differential(size_t local)
      {
        if constexpr (variable_count == 0)
        {
          (void) local;
          return false;
        }
        else if constexpr (Detail::hasStaticDifferential<T>())
        {
          return T::differential(local);
        }
        else
        {
          static_assert(Detail::AlwaysFalse<T>,
                        "EMT components with local variables must define static constexpr bool differential(size_t)");
        }
      }

      static bool differential(const T& component, size_t local)
      {
        if constexpr (variable_count == 0)
        {
          (void) component;
          (void) local;
          return false;
        }
        else if constexpr (Detail::hasRuntimeDifferential<T>())
        {
          return component.differential(local);
        }
        else if constexpr (has_differential)
        {
          (void) component;
          return T::differential(local);
        }
        else
        {
          static_assert(Detail::AlwaysFalse<T>,
                        "EMT components with local variables must define differential(size_t)");
        }
      }

      static constexpr OutputPortSpec outputPort(size_t index)
      {
        if constexpr (output_port_count == 0)
        {
          (void) index;
          return {};
        }
        else
        {
          return T::outputPort(index);
        }
      }
    };

    enum class ComponentJacobianForm
    {
      General,
      Affine
    };

    enum class ComponentJacobianCoefficientUpdate
    {
      PerEvaluation,
      OnStructuralChange,
      Static
    };

    template <class T>
    struct ComponentJacobianTraits
    {
      static constexpr ComponentJacobianForm              form = ComponentJacobianForm::General;
      static constexpr ComponentJacobianCoefficientUpdate coefficient_update =
          ComponentJacobianCoefficientUpdate::PerEvaluation;
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
                      "Component type is not part of this EMT system model data");
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

  } // namespace EMT
} // namespace GridKit
