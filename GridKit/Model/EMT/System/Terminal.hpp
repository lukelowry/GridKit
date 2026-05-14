#pragma once

#include <cstddef>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Component.hpp>

namespace GridKit
{
  namespace EMT
  {
    struct TerminalRef
    {
      ComponentId component;
      size_t      index{INVALID_INDEX<size_t>};
    };

    constexpr TerminalRef ComponentRef::terminal(size_t local) const
    {
      return {id, local};
    }

    template <typename IdxT>
    struct TerminalConnection
    {
      TerminalRef terminal;
      IdxT        bus{INVALID_INDEX<IdxT>};
    };
  } // namespace EMT
} // namespace GridKit
