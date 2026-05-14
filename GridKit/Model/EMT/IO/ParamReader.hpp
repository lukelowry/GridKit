#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/IO/JsonSupport.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Detail
    {
      template <class Data, class Field>
      void assignParamField(Data& data, const nlohmann::json& obj, const Field& field, std::string_view entity)
      {
        using Member = typename Field::Member;

        const auto it = obj.find(std::string(field.key));
        if (it == obj.end())
        {
          if (field.fallback.has_value())
          {
            data.*(field.member) = *field.fallback;
            return;
          }
          Detail::throwCase(Detail::fieldContext(entity, field.key), "required field missing");
        }

        Member value = Detail::readJsonValue<Member>(*it, Detail::fieldContext(entity, field.key));
        if (field.transform != nullptr)
        {
          value = field.transform(value);
        }
        data.*(field.member) = std::move(value);
      }

      template <class Tuple, std::size_t... Is>
      auto paramKeys(const Tuple& fields, std::index_sequence<Is...>)
      {
        return std::array<std::string_view, sizeof...(Is)>{std::get<Is>(fields).key...};
      }
    } // namespace Detail

    template <class Data, class Params>
    Data parseStruct(const nlohmann::json& obj, const Params& fields, std::string_view entity)
    {
      if (!obj.is_object())
      {
        Detail::throwCase(entity, "expected params object");
      }

      constexpr auto field_count = std::tuple_size_v<std::remove_cvref_t<Params>>;
      const auto     keys        = Detail::paramKeys(fields, std::make_index_sequence<field_count>{});
      rejectUnknownKeys(obj, keys, entity);

      Data data{};
      std::apply(
          [&](const auto&... field)
          {
            (Detail::assignParamField(data, obj, field, entity), ...);
          },
          fields);
      return data;
    }
  } // namespace EMT
} // namespace GridKit
