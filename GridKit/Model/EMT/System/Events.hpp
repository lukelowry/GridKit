#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>

#include <GridKit/Model/EMT/Component/Breaker/Breaker.hpp>
#include <GridKit/Model/EMT/Component/BusFault/BusFault.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ComponentT>
    struct ComponentEventTraits
    {
      template <class ActionT>
      static constexpr bool supports()
      {
        return false;
      }

      template <class ActionT>
      static void apply(ComponentT&, const ActionT&)
      {
        throw std::invalid_argument("EMT component does not support this event action");
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

    template <class RealT, typename IdxT>
    struct ComponentEventTraits<Breaker<RealT, IdxT>>
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

    template <class RealT, typename IdxT>
    struct ComponentEventTraits<BusFault<RealT, IdxT>>
    {
      template <class ActionT>
      static constexpr bool supports()
      {
        return std::is_same_v<ActionT, GridKit::Model::Events::Fault>
               || std::is_same_v<ActionT, GridKit::Model::Events::Clear>;
      }

      static void apply(BusFault<RealT, IdxT>&               fault,
                        const GridKit::Model::Events::Fault& action)
      {
        fault.fault(action);
      }

      static void apply(BusFault<RealT, IdxT>&               fault,
                        const GridKit::Model::Events::Clear& action)
      {
        fault.clear(action.phases);
      }

      template <class ActionT>
      static void apply(BusFault<RealT, IdxT>&, const ActionT&)
      {
        throw std::invalid_argument("EMT BusFault supports fault and clear only");
      }
    };

    template <class RealT, typename IdxT>
    struct ComponentStructuralModes<BusFault<RealT, IdxT>>
    {
      template <class Fn>
      static void visit(const BusFault<RealT, IdxT>& fault, Fn&& fn)
      {
        auto cleared = fault;
        cleared.clear(GridKit::Model::Events::PhaseMask::abc());
        fn(cleared);

        auto active = fault;
        active.fault(GridKit::Model::Events::Fault{GridKit::Model::Events::PhaseMask::abc(),
                                                   static_cast<double>(fault.resistance()),
                                                   0.0,
                                                   0.0});
        fn(active);
      }
    };
  } // namespace EMT
} // namespace GridKit
