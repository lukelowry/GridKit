#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    class CaseError : public std::runtime_error
    {
    public:
      using std::runtime_error::runtime_error;
    };

    namespace Detail
    {
      template <class T>
      struct IsPhaseVector : std::false_type
      {
      };

      template <class T>
      struct IsPhaseVector<PhaseVector<T>> : std::true_type
      {
        using value_type = T;
      };

      template <class T>
      struct IsPhaseMatrix : std::false_type
      {
      };

      template <class T>
      struct IsPhaseMatrix<PhaseMatrix<T>> : std::true_type
      {
        using value_type = T;
      };

      template <class T>
      inline constexpr bool IsPhaseVectorV = IsPhaseVector<T>::value && !IsPhaseMatrix<T>::value;

      template <class T>
      inline constexpr bool IsPhaseMatrixV = IsPhaseMatrix<T>::value;

      inline std::string context(std::string_view entity)
      {
        return std::string(entity);
      }

      inline std::string fieldContext(std::string_view entity, std::string_view key)
      {
        if (entity.empty())
        {
          return std::string(key);
        }
        return std::string(entity) + "." + std::string(key);
      }

      [[noreturn]] inline void throwCase(std::string_view context, std::string_view message)
      {
        if (context.empty())
        {
          throw CaseError(std::string(message));
        }
        throw CaseError(std::string(context) + ": " + std::string(message));
      }

      template <class T>
      T readJsonValue(const nlohmann::json& value, std::string_view context);

      inline GridKit::Model::Events::PhaseMask readPhaseMaskToken(std::string_view token, std::string_view context)
      {
        if (token.empty())
        {
          throwCase(context, "phase mask must not be empty");
        }

        std::uint8_t bits = 0u;
        for (const char phase : token)
        {
          std::uint8_t bit = 0u;
          switch (phase)
          {
          case 'a':
            bit = 0b001u;
            break;
          case 'b':
            bit = 0b010u;
            break;
          case 'c':
            bit = 0b100u;
            break;
          default:
            throwCase(context, "phase mask must contain only 'a', 'b', and 'c'");
          }

          if ((bits & bit) != 0u)
          {
            throwCase(context, "phase mask contains a duplicate phase");
          }
          bits = static_cast<std::uint8_t>(bits | bit);
        }
        return GridKit::Model::Events::PhaseMask::fromBits(bits);
      }

      inline GridKit::Model::Events::PhaseMask readPhaseMask(const nlohmann::json& value, std::string_view context)
      {
        try
        {
          if (value.is_string())
          {
            return readPhaseMaskToken(value.get<std::string>(), context);
          }

          if (!value.is_array())
          {
            throwCase(context, "expected phase mask string or array");
          }

          if (value.empty())
          {
            throwCase(context, "phase mask must not be empty");
          }

          std::uint8_t bits = 0u;
          for (const auto& entry : value)
          {
            if (!entry.is_string())
            {
              throwCase(context, "phase mask array entries must be strings");
            }

            const auto token = entry.get<std::string>();
            if (token.size() != 1u)
            {
              throwCase(context, "phase mask array entries must be single phases");
            }

            const auto mask = readPhaseMaskToken(token, context);
            if ((bits & mask.bits()) != 0u)
            {
              throwCase(context, "phase mask contains a duplicate phase");
            }
            bits = static_cast<std::uint8_t>(bits | mask.bits());
          }
          return GridKit::Model::Events::PhaseMask::fromBits(bits);
        }
        catch (const CaseError&)
        {
          throw;
        }
        catch (const nlohmann::json::exception& ex)
        {
          throwCase(context, ex.what());
        }
      }

      template <class T>
      T readPhaseVector(const nlohmann::json& value, std::string_view context)
      {
        using Scalar = typename IsPhaseVector<T>::value_type;
        if (!value.is_array() || value.size() != 3u)
        {
          throwCase(context, "expected phase vector with exactly 3 entries");
        }

        T result{};
        try
        {
          for (std::size_t phase = 0; phase < 3u; ++phase)
          {
            if (value[phase].is_array() || value[phase].is_object())
            {
              throwCase(context, "phase vector entries must be scalar values");
            }
            result[phase] = value[phase].get<Scalar>();
          }
        }
        catch (const CaseError&)
        {
          throw;
        }
        catch (const nlohmann::json::exception& ex)
        {
          throwCase(context, ex.what());
        }
        return result;
      }

      template <class T>
      T readPhaseMatrix(const nlohmann::json& value, std::string_view context)
      {
        using Scalar = typename IsPhaseMatrix<T>::value_type;
        if (!value.is_array() || value.size() != 3u)
        {
          throwCase(context, "expected phase matrix with exactly 3 rows");
        }

        T result{};
        try
        {
          for (std::size_t row = 0; row < 3u; ++row)
          {
            if (!value[row].is_array() || value[row].size() != 3u)
            {
              throwCase(context, "expected phase matrix rows with exactly 3 entries");
            }
            for (std::size_t col = 0; col < 3u; ++col)
            {
              if (value[row][col].is_array() || value[row][col].is_object())
              {
                throwCase(context, "phase matrix entries must be scalar values");
              }
              result[row][col] = value[row][col].get<Scalar>();
            }
          }
        }
        catch (const CaseError&)
        {
          throw;
        }
        catch (const nlohmann::json::exception& ex)
        {
          throwCase(context, ex.what());
        }
        return result;
      }

      template <class T>
      T readJsonValue(const nlohmann::json& value, std::string_view context)
      {
        if constexpr (IsPhaseMatrixV<T>)
        {
          return readPhaseMatrix<T>(value, context);
        }
        else if constexpr (IsPhaseVectorV<T>)
        {
          return readPhaseVector<T>(value, context);
        }
        else if constexpr (std::is_same_v<T, GridKit::Model::Events::PhaseMask>)
        {
          return readPhaseMask(value, context);
        }
        else
        {
          try
          {
            return value.get<T>();
          }
          catch (const nlohmann::json::exception& ex)
          {
            throwCase(context, ex.what());
          }
        }
      }
    } // namespace Detail

    template <class T>
    T require(const nlohmann::json& obj, std::string_view key, std::string_view entity)
    {
      const auto context = Detail::fieldContext(entity, key);
      if (!obj.is_object())
      {
        Detail::throwCase(entity, "expected object");
      }

      const auto it = obj.find(std::string(key));
      if (it == obj.end())
      {
        Detail::throwCase(context, "required field missing");
      }
      return Detail::readJsonValue<T>(*it, context);
    }

    inline void rejectUnknownKeys(const nlohmann::json&             obj,
                                  std::span<const std::string_view> allowed,
                                  std::string_view                  entity)
    {
      if (!obj.is_object())
      {
        Detail::throwCase(entity, "expected object");
      }

      for (const auto& item : obj.items())
      {
        bool known = false;
        for (const auto key : allowed)
        {
          if (item.key() == key)
          {
            known = true;
            break;
          }
        }
        if (!known)
        {
          throw CaseError(std::string(entity) + ": unknown key '" + item.key() + "'");
        }
      }
    }

    template <std::size_t N>
    void rejectUnknownKeys(const nlohmann::json&                  obj,
                           const std::array<std::string_view, N>& allowed,
                           std::string_view                       entity)
    {
      rejectUnknownKeys(obj, std::span<const std::string_view>{allowed.data(), allowed.size()}, entity);
    }

    inline void rejectUnknownKeys(const nlohmann::json&                   obj,
                                  std::initializer_list<std::string_view> allowed,
                                  std::string_view                        entity)
    {
      rejectUnknownKeys(obj, std::span<const std::string_view>{allowed.begin(), allowed.size()}, entity);
    }
  } // namespace EMT
} // namespace GridKit
