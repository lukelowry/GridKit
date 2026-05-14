#pragma once

#include <cstddef>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Component.hpp>

namespace GridKit
{
  namespace EMT
  {
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

    constexpr InputRef ComponentRef::input(size_t local) const
    {
      return {id, local};
    }

    constexpr OutputRef ComponentRef::output(size_t local) const
    {
      return {id, local};
    }

    struct PortConnection
    {
      OutputRef output;
      InputRef  input;
    };
  } // namespace EMT
} // namespace GridKit
