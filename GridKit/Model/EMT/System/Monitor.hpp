#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/Bus/Bus.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class BusMonitorVariable
    {
      va,
      vb,
      vc,
      dva,
      dvb,
      dvc,
      ifa,
      ifb,
      ifc
    };

    inline std::optional<BusMonitorVariable> resolveBusMonitorVariable(std::string_view name)
    {
      if (name == "va")
      {
        return BusMonitorVariable::va;
      }
      if (name == "vb")
      {
        return BusMonitorVariable::vb;
      }
      if (name == "vc")
      {
        return BusMonitorVariable::vc;
      }
      if (name == "dva")
      {
        return BusMonitorVariable::dva;
      }
      if (name == "dvb")
      {
        return BusMonitorVariable::dvb;
      }
      if (name == "dvc")
      {
        return BusMonitorVariable::dvc;
      }
      if (name == "ifa")
      {
        return BusMonitorVariable::ifa;
      }
      if (name == "ifb")
      {
        return BusMonitorVariable::ifb;
      }
      if (name == "ifc")
      {
        return BusMonitorVariable::ifc;
      }
      return std::nullopt;
    }

    struct MonitorEntry
    {
      std::string_view name;
      bool             derivative;
      std::size_t      local;
    };

    template <class Enum>
    constexpr std::array<MonitorEntry, 6> phaseCurrentMonitors()
    {
      return {{{"ia", false, 0u},
               {"ib", false, 1u},
               {"ic", false, 2u},
               {"dia", true, 0u},
               {"dib", true, 1u},
               {"dic", true, 2u}}};
    }

    template <class Entries>
    std::optional<std::size_t> resolveMonitor(const Entries& entries, std::string_view name)
    {
      for (std::size_t index = 0; index < entries.size(); ++index)
      {
        if (entries[index].name == name)
        {
          return index;
        }
      }
      return std::nullopt;
    }

    template <class Context, class Entries>
    void bindStateMonitors(Context& ctx, const Entries& entries, std::size_t raw)
    {
      if (raw >= entries.size())
      {
        throw std::invalid_argument("Invalid EMT component monitor variable");
      }

      const auto& entry = entries[raw];
      if (entry.derivative)
      {
        ctx.addDerivative(std::string(entry.name), entry.local);
      }
      else
      {
        ctx.addState(std::string(entry.name), entry.local);
      }
    }

    template <class Derived, class Enum>
    struct StateMonitorTable
    {
      using Variable = Enum;

      template <class Context, class Component>
      static void bind(Context& ctx, const Component&, std::size_t raw)
      {
        bindStateMonitors(ctx, Derived::entries, raw);
      }

      static std::optional<std::size_t> resolve(std::string_view name)
      {
        return resolveMonitor(Derived::entries, name);
      }
    };

    template <typename IdxT>
    struct BusMonitorRequest
    {
      IdxT                            bus{INVALID_INDEX<IdxT>};
      std::string                     label;
      std::vector<BusMonitorVariable> variables;
    };

    template <class EntityT>
    struct BusMonitorTraits;

    template <class RealT, typename IdxT>
    struct BusMonitorTraits<Bus<RealT, IdxT>>
    {
      using Variable = BusMonitorVariable;

      template <class Context>
      static void bind(Context& ctx, const Bus<RealT, IdxT>& bus, Variable variable)
      {
        switch (variable)
        {
        case Variable::va:
          ctx.addVoltage("va", IdxT{0});
          return;
        case Variable::vb:
          ctx.addVoltage("vb", IdxT{1});
          return;
        case Variable::vc:
          ctx.addVoltage("vc", IdxT{2});
          return;
        case Variable::dva:
          ctx.addVoltageDerivative("dva", IdxT{0});
          return;
        case Variable::dvb:
          ctx.addVoltageDerivative("dvb", IdxT{1});
          return;
        case Variable::dvc:
          ctx.addVoltageDerivative("dvc", IdxT{2});
          return;
        case Variable::ifa:
          bindFaultCurrent(ctx, bus, "ifa", IdxT{0});
          return;
        case Variable::ifb:
          bindFaultCurrent(ctx, bus, "ifb", IdxT{1});
          return;
        case Variable::ifc:
          bindFaultCurrent(ctx, bus, "ifc", IdxT{2});
          return;
        }

        throw std::invalid_argument("Invalid EMT bus monitor variable");
      }

      static std::optional<Variable> resolve(std::string_view name)
      {
        return resolveBusMonitorVariable(name);
      }

    private:
      template <class Context>
      static void bindFaultCurrent(Context&                ctx,
                                   const Bus<RealT, IdxT>& bus,
                                   std::string             label,
                                   IdxT                    phase)
      {
        const IdxT  index = ctx.voltageIndex(phase);
        const auto* y     = ctx.y();
        ctx.add(std::move(label),
                [&bus, y, index, phase]()
                {
                  return bus.faultCurrent((*y)[static_cast<size_t>(index)], static_cast<size_t>(phase));
                });
      }
    };

    template <class ComponentT>
    struct ComponentMonitorTraits;
  } // namespace EMT
} // namespace GridKit
