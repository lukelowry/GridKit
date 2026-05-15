#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <magic_enum/magic_enum.hpp>

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

    template <class Enum>
    std::optional<Enum> resolveMonitorVariable(std::string_view name)
    {
      return magic_enum::enum_cast<Enum>(name, magic_enum::case_insensitive);
    }

    inline std::optional<BusMonitorVariable> resolveBusMonitorVariable(std::string_view name)
    {
      return resolveMonitorVariable<BusMonitorVariable>(name);
    }

    template <class Enum>
    struct MonitorEntry
    {
      using Variable = Enum;

      Enum             variable;
      std::string_view name;
      bool             derivative;
      std::size_t      local;
    };

    template <class Enum>
    constexpr std::array<MonitorEntry<Enum>, 6> phaseCurrentMonitors()
    {
      return {{{Enum::ia, "ia", false, 0u},
               {Enum::ib, "ib", false, 1u},
               {Enum::ic, "ic", false, 2u},
               {Enum::dia, "dia", true, 0u},
               {Enum::dib, "dib", true, 1u},
               {Enum::dic, "dic", true, 2u}}};
    }

    template <class Entries>
    std::optional<typename Entries::value_type::Variable> resolveMonitor(const Entries& entries, std::string_view name)
    {
      using Variable    = typename Entries::value_type::Variable;
      const auto parsed = resolveMonitorVariable<Variable>(name);
      if (!parsed.has_value())
      {
        return std::nullopt;
      }

      for (const auto& entry : entries)
      {
        if (entry.variable == *parsed)
        {
          return parsed;
        }
      }
      return std::nullopt;
    }

    template <class Context, class Entries>
    void bindStateMonitors(Context& ctx, const Entries& entries, typename Entries::value_type::Variable variable)
    {
      for (const auto& entry : entries)
      {
        if (entry.variable == variable)
        {
          if (entry.derivative)
          {
            ctx.addDerivative(std::string(entry.name), entry.local);
          }
          else
          {
            ctx.addState(std::string(entry.name), entry.local);
          }
          return;
        }
      }

      throw std::invalid_argument("Invalid EMT component monitor variable");
    }

    template <class Derived, class Enum>
    struct StateMonitorTable
    {
      using Variable = Enum;

      template <class Context, class Component>
      static void bind(Context& ctx, const Component&, Variable variable)
      {
        bindStateMonitors(ctx, Derived::entries, variable);
      }

      static std::optional<Variable> resolve(std::string_view name)
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
