#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Component/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumped.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRL.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Model/EMT/ComponentDataJSONParser.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type, std::size_t N = 3>
    class SystemModel final : public PhasorDynamics::SystemModel<scalar_type, index_type>
    {
    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using BaseT      = PhasorDynamics::SystemModel<ScalarT, IdxT>;
      using RealT      = typename BaseT::RealT;
      using ModelDataT = EMT::SystemModelData<RealT, IdxT, N>;
      using BusT       = EMT::Bus<ScalarT, IdxT>;
      using ComponentT = PhasorDynamics::Component<ScalarT, IdxT>;
      using SourceT    = EMT::VoltageSource<ScalarT, IdxT, N>;
      using LoadT      = EMT::LoadRL<ScalarT, IdxT>;
      using LineT      = EMT::LineLumped<ScalarT, IdxT, N>;

      SystemModel() = default;

      explicit SystemModel(const ModelDataT& data)
      {
        build(data);
      }

    private:
      void build(const ModelDataT& data)
      {
        for (const auto& bus_data : data.bus)
        {
          addOwnedBus(bus_data);
        }

        for (const auto& source_data : data.voltage_source)
        {
          const auto bus_id = requiredPort(source_data.ports,
                                           VoltageSourcePorts::bus,
                                           context(source_data));
          addOwnedComponent<SourceT>(bus(bus_id), source_data);
        }

        for (const auto& load_data : data.load_rl)
        {
          const auto bus_id = requiredPort(load_data.ports,
                                           LoadRLPorts::bus,
                                           context(load_data));
          addOwnedComponent<LoadT>(bus(bus_id), load_data);
        }

        for (const auto& line_data : data.line_lumped)
        {
          const auto bus1_id = requiredPort(line_data.ports,
                                            LineLumpedPorts::bus1,
                                            context(line_data));
          const auto bus2_id = requiredPort(line_data.ports,
                                            LineLumpedPorts::bus2,
                                            context(line_data));
          addOwnedComponent<LineT>(bus(bus1_id), bus(bus2_id), line_data);
        }
      }

      void addOwnedBus(const typename ModelDataT::BusDataT& data)
      {
        if (buses_by_id_.find(data.bus_id) != buses_by_id_.end())
        {
          std::stringstream ss;
          ss << "EMT SystemModel: duplicate bus id " << data.bus_id;
          throw std::invalid_argument(ss.str());
        }

        auto  values = std::vector<RealT>(data.v0.begin(), data.v0.end());
        auto  owned  = std::make_unique<BusT>(data.bus_id, std::move(values));
        auto* raw    = owned.get();

        buses_by_id_[data.bus_id] = raw;
        BaseT::addBus(raw);
        owned_buses_.push_back(std::move(owned));
      }

      BusT* bus(IdxT bus_id) const
      {
        auto it = buses_by_id_.find(bus_id);
        if (it == buses_by_id_.end())
        {
          std::stringstream ss;
          ss << "EMT SystemModel: unknown bus id " << bus_id;
          throw std::invalid_argument(ss.str());
        }

        return it->second;
      }

      template <typename ComponentModelT, typename... Args>
      void addOwnedComponent(Args&&... args)
      {
        auto component = std::make_unique<ComponentModelT>(std::forward<Args>(args)...);
        BaseT::addComponent(component.get());
        owned_components_.push_back(std::move(component));
      }

      template <typename DataT>
      static std::string context(const DataT& data)
      {
        return componentContext(data);
      }

      std::vector<std::unique_ptr<BusT>>       owned_buses_;
      std::vector<std::unique_ptr<ComponentT>> owned_components_;
      std::map<IdxT, BusT*>                    buses_by_id_;
    };
  } // namespace EMT
} // namespace GridKit
