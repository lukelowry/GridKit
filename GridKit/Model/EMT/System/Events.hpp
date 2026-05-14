#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Component/Breaker/Breaker.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class EntityT>
    struct EventTraits
    {
      template <class ActionT>
      static constexpr bool supports()
      {
        return false;
      }

      template <class ActionT>
      static void apply(EntityT&, const ActionT&)
      {
        throw std::invalid_argument("EMT entity does not support scheduled event action");
      }

      template <class ActionT>
      static void prepare(EntityT&, const ActionT&)
      {
      }
    };

    template <class ComponentT>
    struct ComponentStructuralModes
    {
      template <class Fn>
      static void visit(const ComponentT& component, Fn&& fn)
      {
        fn(component);
      }
    };

    template <class EntityT>
    struct ResidualTraits
    {
      template <class Context>
      static void evaluate(Context&, const EntityT&)
      {
      }
    };

    template <class RealT, typename IdxT>
    struct EventTraits<Bus<RealT, IdxT>>
    {
      template <class ActionT>
      static constexpr bool supports()
      {
        return std::is_same_v<ActionT, GridKit::Model::Events::Fault>
               || std::is_same_v<ActionT, GridKit::Model::Events::Clear>;
      }

      static void apply(Bus<RealT, IdxT>&                    bus,
                        const GridKit::Model::Events::Fault& action)
      {
        bus.fault(action);
      }

      static void apply(Bus<RealT, IdxT>&                    bus,
                        const GridKit::Model::Events::Clear& action)
      {
        bus.clear(action.phases);
      }

      static void prepare(Bus<RealT, IdxT>& bus, const GridKit::Model::Events::Fault&)
      {
        bus.prepareFaultStructure();
      }

      static void prepare(Bus<RealT, IdxT>& bus, const GridKit::Model::Events::Clear&)
      {
        bus.prepareFaultStructure();
      }

      template <class ActionT>
      static void apply(Bus<RealT, IdxT>&, const ActionT&)
      {
        throw std::invalid_argument("EMT bus supports fault and clear only");
      }
    };

    template <class RealT, typename IdxT>
    struct ResidualTraits<Bus<RealT, IdxT>>
    {
      template <class Context>
      static void evaluate(Context& ctx, const Bus<RealT, IdxT>& bus)
      {
        for (IdxT phase = 0; phase < static_cast<IdxT>(ctx.phaseCount()); ++phase)
        {
          ctx.addCurrent(phase, bus.residualCurrent(ctx.voltage(phase), static_cast<size_t>(phase)));
        }
      }
    };

    template <class RealT, typename IdxT>
    struct EventTraits<Breaker<RealT, IdxT>>
    {
      template <class ActionT>
      static constexpr bool supports()
      {
        return std::is_same_v<ActionT, GridKit::Model::Events::Open>
               || std::is_same_v<ActionT, GridKit::Model::Events::Close>;
      }

      static void apply(Breaker<RealT, IdxT>&               breaker,
                        const GridKit::Model::Events::Open& action)
      {
        breaker.open(action.phases);
      }

      static void apply(Breaker<RealT, IdxT>&                breaker,
                        const GridKit::Model::Events::Close& action)
      {
        breaker.close(action.phases);
      }

      static void prepare(Breaker<RealT, IdxT>&, const GridKit::Model::Events::Open&)
      {
      }

      static void prepare(Breaker<RealT, IdxT>&, const GridKit::Model::Events::Close&)
      {
      }

      template <class ActionT>
      static void apply(Breaker<RealT, IdxT>&, const ActionT&)
      {
        throw std::invalid_argument("EMT Breaker supports open and close only");
      }
    };

    template <class RealT, typename IdxT>
    struct ComponentStructuralModes<Breaker<RealT, IdxT>>
    {
      template <class Fn>
      static void visit(const Breaker<RealT, IdxT>& breaker, Fn&& fn)
      {
        auto open = breaker;
        open.open(GridKit::Model::Events::PhaseMask::abc());
        fn(open);

        auto closed = breaker;
        closed.close(GridKit::Model::Events::PhaseMask::abc());
        fn(closed);
      }
    };
  } // namespace EMT
} // namespace GridKit
