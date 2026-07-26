#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace GridKit::EMT
{
  struct InitialStateVariable
  {
    std::string                name;
    std::optional<std::size_t> index;
    std::size_t                local_offset;
  };

  class InitialStateLayout
  {
  public:
    virtual ~InitialStateLayout() = default;

    virtual void appendInitialStateVariables(
        std::vector<InitialStateVariable>& variables) const = 0;
  };
} // namespace GridKit::EMT
