#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/System/Component.hpp>
#include <GridKit/Model/EMT/System/Port.hpp>
#include <GridKit/Model/EMT/System/Terminal.hpp>

namespace GridKit
{
  namespace EMT
  {
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
    } // namespace Detail

    template <class... Ts>
    class ComponentStore
    {
    public:
      template <class T>
      ComponentRef add(T component)
      {
        using ComponentT = std::decay_t<T>;
        static_assert(Detail::Contains<ComponentT, Ts...>,
                      "Component type is not part of this EMT network");
        auto&        components = std::get<std::vector<ComponentT>>(components_);
        ComponentRef ref{{Detail::TypeIndex<ComponentT, Ts...>::value, components.size()}};
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
