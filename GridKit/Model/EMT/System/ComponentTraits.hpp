#pragma once

#include <cstddef>

#include <GridKit/Model/EMT/System/Port.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Detail
    {
      template <class T>
      consteval bool componentDirectJacobian()
      {
        if constexpr (requires { T::direct_jacobian; })
        {
          return T::direct_jacobian;
        }
        else
        {
          return false;
        }
      }
    } // namespace Detail

    template <class T>
    struct ComponentTraits
    {
      static constexpr size_t variables       = T::variables;
      static constexpr size_t equations       = T::equations;
      static constexpr size_t terminals       = T::terminals;
      static constexpr size_t inputs          = T::inputs;
      static constexpr size_t outputs         = T::outputs;
      static constexpr bool   direct_jacobian = Detail::componentDirectJacobian<T>();

      static constexpr bool differential(size_t local)
      {
        if constexpr (requires { T::differential(local); })
        {
          return T::differential(local);
        }
        else
        {
          return true;
        }
      }

      static constexpr OutputSpec output(size_t index)
      {
        if constexpr (outputs == 0)
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
  } // namespace EMT
} // namespace GridKit
