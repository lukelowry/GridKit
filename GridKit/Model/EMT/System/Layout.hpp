#pragma once

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/System/Components.hpp>

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
      IdxT electrical_port_offset{INVALID_INDEX<IdxT>};
      IdxT electrical_port_count{0};
      IdxT input_port_offset{INVALID_INDEX<IdxT>};
      IdxT input_port_count{0};
    };

    template <typename IdxT>
    class Layout
    {
    public:
      static constexpr IdxT phases = 3;

      void reset(IdxT buses, size_t component_types)
      {
        buses_                 = buses;
        variable_count_        = phases * buses;
        equation_count_        = phases * buses;
        electrical_port_count_ = 0;
        input_port_count_      = 0;
        component_layouts_.assign(component_types, {});
        port_bus_ids_.clear();
        input_port_variable_ids_.clear();
      }

      ComponentLayout<IdxT> appendComponent(size_t type,
                                            IdxT   variables,
                                            IdxT   equations,
                                            IdxT   electrical_ports,
                                            IdxT   input_ports)
      {
        if (type >= component_layouts_.size())
        {
          throw std::out_of_range("EMT layout type index is out of range");
        }

        ComponentLayout<IdxT> slot{variable_count_,
                                   equation_count_,
                                   variables,
                                   equations,
                                   electrical_port_count_,
                                   electrical_ports,
                                   input_port_count_,
                                   input_ports};
        component_layouts_[type].push_back(slot);
        variable_count_        += variables;
        equation_count_        += equations;
        electrical_port_count_ += electrical_ports;
        input_port_count_      += input_ports;
        return slot;
      }

      void allocateConnections()
      {
        port_bus_ids_.assign(static_cast<size_t>(electrical_port_count_), INVALID_INDEX<IdxT>);
        input_port_variable_ids_.assign(static_cast<size_t>(input_port_count_), INVALID_INDEX<IdxT>);
      }

      IdxT busCount() const
      {
        return buses_;
      }

      IdxT size() const
      {
        return variable_count_;
      }

      IdxT equationCount() const
      {
        return equation_count_;
      }

      IdxT electricalPortCount() const
      {
        return electrical_port_count_;
      }

      IdxT inputPortCount() const
      {
        return input_port_count_;
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
        return at(id);
      }

      template <class RefT>
      const ComponentLayout<IdxT>& component(const RefT& ref) const
        requires requires { ref.id; }
      {
        return component(ref.id);
      }

      void connectPort(ComponentId component_id, size_t port, IdxT bus)
      {
        const auto& slot = component(component_id);
        if (port >= static_cast<size_t>(slot.electrical_port_count))
        {
          throw std::invalid_argument("EMT electrical port index is out of range");
        }

        IdxT& bus_id = port_bus_ids_[static_cast<size_t>(slot.electrical_port_offset) + port];
        if (bus_id != INVALID_INDEX<IdxT>)
        {
          throw std::invalid_argument("EMT electrical port is connected more than once");
        }
        bus_id = bus;
      }

      void connectInputPort(ComponentId component_id, size_t input_port, IdxT variable)
      {
        const auto& slot = component(component_id);
        if (input_port >= static_cast<size_t>(slot.input_port_count))
        {
          throw std::invalid_argument("EMT input port index is out of range");
        }

        IdxT& input_variable =
            input_port_variable_ids_[static_cast<size_t>(slot.input_port_offset) + input_port];
        if (input_variable != INVALID_INDEX<IdxT>)
        {
          throw std::invalid_argument("EMT input port is connected more than once");
        }
        input_variable = variable;
      }

      void validateConnections() const
      {
        for (const auto& layouts_for_type : component_layouts_)
        {
          for (const auto& slot : layouts_for_type)
          {
            for (IdxT local = 0; local < slot.electrical_port_count; ++local)
            {
              if (port_bus_ids_[static_cast<size_t>(slot.electrical_port_offset + local)] == INVALID_INDEX<IdxT>)
              {
                throw std::invalid_argument("EMT component electrical port is not connected");
              }
            }
            for (IdxT local = 0; local < slot.input_port_count; ++local)
            {
              if (input_port_variable_ids_[static_cast<size_t>(slot.input_port_offset + local)] == INVALID_INDEX<IdxT>)
              {
                throw std::invalid_argument("EMT input port is not connected");
              }
            }
          }
        }
      }

      std::span<const IdxT> portBuses(ComponentId id) const
      {
        const auto& slot = component(id);
        if (slot.electrical_port_count == 0)
        {
          return {};
        }
        return {port_bus_ids_.data() + slot.electrical_port_offset, static_cast<size_t>(slot.electrical_port_count)};
      }

      std::span<const IdxT> inputPortVariables(ComponentId id) const
      {
        const auto& slot = component(id);
        if (slot.input_port_count == 0)
        {
          return {};
        }
        return {input_port_variable_ids_.data() + slot.input_port_offset,
                static_cast<size_t>(slot.input_port_count)};
      }

      const std::vector<std::vector<ComponentLayout<IdxT>>>& componentLayouts() const
      {
        return component_layouts_;
      }

    private:
      const ComponentLayout<IdxT>& at(ComponentId id) const
      {
        if (id.type >= component_layouts_.size() || id.index >= component_layouts_[id.type].size())
        {
          throw std::out_of_range("EMT component id is not present in the layout");
        }
        return component_layouts_[id.type][id.index];
      }

      IdxT                                            buses_{0};
      IdxT                                            variable_count_{0};
      IdxT                                            equation_count_{0};
      IdxT                                            electrical_port_count_{0};
      IdxT                                            input_port_count_{0};
      std::vector<std::vector<ComponentLayout<IdxT>>> component_layouts_;
      std::vector<IdxT>                               port_bus_ids_;
      std::vector<IdxT>                               input_port_variable_ids_;
    };
  } // namespace EMT
} // namespace GridKit
