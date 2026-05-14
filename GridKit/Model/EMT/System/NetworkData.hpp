#pragma once

#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/System/ComponentStore.hpp>
#include <GridKit/Model/EMT/System/Port.hpp>
#include <GridKit/Model/EMT/System/Terminal.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT, class... ComponentTs>
    struct NetworkData
    {
      using scalar_type          = RealT;
      using index_type           = IdxT;
      using component_store_type = ComponentStore<ComponentTs...>;

      std::vector<Bus<RealT, IdxT>>         buses;
      component_store_type                  components;
      std::vector<TerminalConnection<IdxT>> terminal_connections;
      std::vector<PortConnection>           port_connections;

      IdxT addBus(BusData<RealT, IdxT> data)
      {
        const IdxT id = static_cast<IdxT>(buses.size());
        buses.emplace_back(data);
        return id;
      }

      template <class T>
      ComponentRef add(T component)
      {
        return components.add(std::move(component));
      }

      void connect(TerminalRef terminal, IdxT bus)
      {
        terminal_connections.push_back({terminal, bus});
      }

      void connect(OutputRef output, InputRef input)
      {
        port_connections.push_back({output, input});
      }
    };
  } // namespace EMT
} // namespace GridKit
