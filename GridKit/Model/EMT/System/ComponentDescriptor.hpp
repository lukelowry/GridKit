#pragma once

#include <concepts>
#include <optional>
#include <string_view>
#include <utility>

#include <GridKit/Model/EMT/System/Components.hpp>
#include <GridKit/Model/EMT/System/Monitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class Component>
    struct ComponentDescriptor;

    template <class DataT, class MemberT>
    struct ParamField
    {
      using Data   = DataT;
      using Member = MemberT;

      std::string_view key;
      Member Data::* member;
      Member (*transform)(Member) = nullptr;
      std::optional<Member> fallback{};
    };

    template <class Data, class M>
    constexpr ParamField<Data, M> field(std::string_view key, M Data::* member)
    {
      return {key, member};
    }

    template <class Data, class M>
    constexpr ParamField<Data, M> field(std::string_view key, M Data::* member, M (*transform)(M))
    {
      return {key, member, transform};
    }

    template <class Data, class M>
    constexpr ParamField<Data, M> optionalField(std::string_view key, M Data::* member, M fallback)
    {
      return {key, member, nullptr, std::move(fallback)};
    }

    template <class T>
    concept Describable = requires {
      typename ComponentDescriptor<T>::Data;
      { ComponentDescriptor<T>::class_name } -> std::convertible_to<std::string_view>;
      ComponentDescriptor<T>::electrical_ports;
      ComponentDescriptor<T>::input_ports;
      ComponentDescriptor<T>::output_ports;
      ComponentDescriptor<T>::params;
    };
  } // namespace EMT
} // namespace GridKit
