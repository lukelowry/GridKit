#pragma once

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Component.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename IdxT>
    struct ComponentLayout
    {
      IdxT variable_offset{INVALID_INDEX<IdxT>};
      IdxT equation_offset{INVALID_INDEX<IdxT>};
      IdxT variable_count{0};
      IdxT equation_count{0};
      IdxT terminal_offset{INVALID_INDEX<IdxT>};
      IdxT terminal_count{0};
      IdxT input_offset{INVALID_INDEX<IdxT>};
      IdxT input_count{0};
    };

    template <typename IdxT>
    class Layout
    {
    public:
      static constexpr IdxT phases = 3;

      void reset(IdxT buses)
      {
        buses_          = buses;
        variable_count_ = phases * buses;
        equation_count_ = phases * buses;
        terminal_count_ = 0;
        input_count_    = 0;
        component_layouts_.clear();
      }

      void setComponentTypes(size_t component_types)
      {
        component_layouts_.assign(component_types, {});
      }

      ComponentLayout<IdxT> appendComponent(size_t type,
                                            IdxT   variables,
                                            IdxT   equations,
                                            IdxT   terminals,
                                            IdxT   inputs)
      {
        return append(component_layouts_, type, variables, equations, terminals, inputs);
      }

      IdxT busCount() const
      {
        return buses_;
      }

      IdxT size() const
      {
        return variable_count_;
      }

      IdxT equations() const
      {
        return equation_count_;
      }

      IdxT terminalCount() const
      {
        return terminal_count_;
      }

      IdxT inputCount() const
      {
        return input_count_;
      }

      IdxT busVariable(IdxT bus, IdxT phase) const
      {
        return phases * bus + phase;
      }

      IdxT busEquation(IdxT bus, IdxT phase) const
      {
        return phases * bus + phase;
      }

      const ComponentLayout<IdxT>& component(ComponentId id) const
      {
        return at(component_layouts_, id);
      }

      template <class RefT>
      const ComponentLayout<IdxT>& component(const RefT& ref) const
        requires requires { ref.id; }
      {
        return component(ref.id);
      }

      std::span<const IdxT> terminals(const std::vector<IdxT>& values, ComponentId id) const
      {
        const auto& slot = component(id);
        if (slot.terminal_count == 0)
        {
          return {};
        }
        return {values.data() + slot.terminal_offset, slot.terminal_count};
      }

      std::span<const IdxT> inputs(const std::vector<IdxT>& values, ComponentId id) const
      {
        const auto& slot = component(id);
        if (slot.input_count == 0)
        {
          return {};
        }
        return {values.data() + slot.input_offset, slot.input_count};
      }

      const std::vector<std::vector<ComponentLayout<IdxT>>>& componentLayouts() const
      {
        return component_layouts_;
      }

    private:
      ComponentLayout<IdxT> append(std::vector<std::vector<ComponentLayout<IdxT>>>& layouts,
                                   size_t                                           type,
                                   IdxT                                             variables,
                                   IdxT                                             equations,
                                   IdxT                                             terminals,
                                   IdxT                                             inputs)
      {
        if (type >= layouts.size())
        {
          throw std::out_of_range("EMT layout type index is out of range");
        }

        ComponentLayout<IdxT> slot{variable_count_,
                                   equation_count_,
                                   variables,
                                   equations,
                                   terminal_count_,
                                   terminals,
                                   input_count_,
                                   inputs};
        layouts[type].push_back(slot);
        variable_count_ += variables;
        equation_count_ += equations;
        terminal_count_ += terminals;
        input_count_    += inputs;
        return slot;
      }

      const ComponentLayout<IdxT>& at(const std::vector<std::vector<ComponentLayout<IdxT>>>& layouts,
                                      ComponentId                                            id) const
      {
        if (id.type >= layouts.size() || id.index >= layouts[id.type].size())
        {
          throw std::out_of_range("EMT component id is not present in the layout");
        }
        return layouts[id.type][id.index];
      }

      IdxT                                            buses_{0};
      IdxT                                            variable_count_{0};
      IdxT                                            equation_count_{0};
      IdxT                                            terminal_count_{0};
      IdxT                                            input_count_{0};
      std::vector<std::vector<ComponentLayout<IdxT>>> component_layouts_;
    };
  } // namespace EMT
} // namespace GridKit
