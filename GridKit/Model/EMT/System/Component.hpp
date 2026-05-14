#pragma once

#include <cstddef>

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

    constexpr bool operator==(const ComponentId& lhs, const ComponentId& rhs)
    {
      return lhs.type == rhs.type && lhs.index == rhs.index;
    }
  } // namespace EMT
} // namespace GridKit
