#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/EMT/Branch/BranchLumpedConstant/BranchLumpedConstant.hpp>
#include <GridKit/Model/EMT/Component/LoadRL/LoadRL.hpp>
#include <GridKit/Model/EMT/Component/VoltageSource/VoltageSource.hpp>

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
      dvc
    };

    enum class LoadRLMonitorVariable
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    enum class BranchLumpedConstantMonitorVariable
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    enum class VoltageSourceMonitorVariable
    {
      ia,
      ib,
      ic
    };

    template <typename IdxT>
    struct BusMonitorRequest
    {
      IdxT                            bus{INVALID_INDEX<IdxT>};
      std::string                     label;
      std::vector<BusMonitorVariable> variables;
    };

    template <class ComponentT>
    struct ComponentMonitorTraits;

    template <class RealT, typename IdxT>
    struct ComponentMonitorTraits<LoadRL<RealT, IdxT>>
    {
      using Variable = LoadRLMonitorVariable;

      template <class Context>
      static void bind(Context& ctx, const LoadRL<RealT, IdxT>&, size_t raw)
      {
        switch (static_cast<Variable>(raw))
        {
        case Variable::ia:
          ctx.addState("ia", 0);
          break;
        case Variable::ib:
          ctx.addState("ib", 1);
          break;
        case Variable::ic:
          ctx.addState("ic", 2);
          break;
        case Variable::dia:
          ctx.addDerivative("dia", 0);
          break;
        case Variable::dib:
          ctx.addDerivative("dib", 1);
          break;
        case Variable::dic:
          ctx.addDerivative("dic", 2);
          break;
        default:
          throw std::invalid_argument("Invalid EMT LoadRL monitor variable");
        }
      }
    };

    template <class RealT, typename IdxT>
    struct ComponentMonitorTraits<BranchLumpedConstant<RealT, IdxT>>
    {
      using Variable = BranchLumpedConstantMonitorVariable;

      template <class Context>
      static void bind(Context& ctx, const BranchLumpedConstant<RealT, IdxT>&, size_t raw)
      {
        switch (static_cast<Variable>(raw))
        {
        case Variable::ia:
          ctx.addState("ia", 0);
          break;
        case Variable::ib:
          ctx.addState("ib", 1);
          break;
        case Variable::ic:
          ctx.addState("ic", 2);
          break;
        case Variable::dia:
          ctx.addDerivative("dia", 0);
          break;
        case Variable::dib:
          ctx.addDerivative("dib", 1);
          break;
        case Variable::dic:
          ctx.addDerivative("dic", 2);
          break;
        default:
          throw std::invalid_argument("Invalid EMT BranchLumpedConstant monitor variable");
        }
      }
    };

    template <class RealT, typename IdxT>
    struct ComponentMonitorTraits<VoltageSource<RealT, IdxT>>
    {
      using Variable = VoltageSourceMonitorVariable;

      template <class Context>
      static void bind(Context& ctx, const VoltageSource<RealT, IdxT>& source, size_t raw)
      {
        IdxT        phase = 0;
        const char* label = nullptr;
        switch (static_cast<Variable>(raw))
        {
        case Variable::ia:
          phase = 0;
          label = "ia";
          break;
        case Variable::ib:
          phase = 1;
          label = "ib";
          break;
        case Variable::ic:
          phase = 2;
          label = "ic";
          break;
        default:
          throw std::invalid_argument("Invalid EMT VoltageSource monitor variable");
        }

        const auto data        = source.data();
        const auto bus_voltage = ctx.terminalVoltageIndex(0, phase);
        const auto y           = ctx.y();
        const auto time        = ctx.time();

        ctx.add(label,
                [data, y, time, bus_voltage, phase]()
                {
                  const RealT sqrt2 = std::sqrt(RealT{2.0});
                  const RealT e     = sqrt2 * data.e[phase] * std::cos(data.omega0 * (*time) + data.phi[phase]);
                  const RealT v     = static_cast<RealT>((*y)[static_cast<size_t>(bus_voltage)]);
                  return (e - v) / data.r[phase];
                });
      }
    };
  } // namespace EMT
} // namespace GridKit
