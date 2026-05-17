#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/IO/FitFile.hpp>
#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/EMT/Math/RationalApprox/RationalApprox.hpp>
#include <GridKit/Model/EMT/Math/RationalApprox/RationalApproxData.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BranchFrequencyDependentData
    {
      RealT                              length{}; // m
      std::vector<std::string>          phase_order;
      std::filesystem::path             yc_fit_file;
      std::string                       yc_fit_name;
      Math::RationalApproxData<RealT>   yc;
    };

    template <class RealT, typename IdxT>
    class BranchFrequencyDependent
    {
    public:
      using Data = BranchFrequencyDependentData<RealT, IdxT>;

      static constexpr size_t variable_count        = dynamic_component_count;
      static constexpr size_t equation_count        = dynamic_component_count;
      static constexpr size_t electrical_port_count = 2;
      static constexpr size_t input_port_count      = 0;
      static constexpr size_t output_port_count     = 0;

      static constexpr size_t from = 0;
      static constexpr size_t to   = 1;

      explicit BranchFrequencyDependent(Data data)
        : data_{std::move(data)},
          yc_from_{data_.yc},
          yc_to_{data_.yc}
      {
        if (!std::isfinite(data_.length) || data_.length <= RealT{0.0})
        {
          throw std::invalid_argument("EMT BranchFrequencyDependent length must be positive");
        }
        requirePhaseOrder(data_.phase_order, "EMT BranchFrequencyDependent phase_order");
        if (yc_from_.dimension() != 3u)
        {
          throw std::invalid_argument("EMT BranchFrequencyDependent Yc fit dimension must be 3");
        }
      }

      static BranchFrequencyDependent fromJson(const nlohmann::json&        params,
                                               const std::filesystem::path& base_dir,
                                               std::string_view             entity)
      {
        if (!params.is_object())
        {
          Detail::throwCase(entity, "expected params object");
        }
        rejectUnknownKeys(params, {"length", "phase_order", "yc"}, entity);

        Data data;
        data.length      = require<RealT>(params, "length", entity);
        data.phase_order = require<std::vector<std::string>>(params, "phase_order", entity);
        requirePhaseOrder(data.phase_order, Detail::fieldContext(entity, "phase_order"));

        const auto yc_it = params.find("yc");
        if (yc_it == params.end())
        {
          Detail::throwCase(Detail::fieldContext(entity, "yc"), "required field missing");
        }
        if (!yc_it->is_object())
        {
          Detail::throwCase(Detail::fieldContext(entity, "yc"), "expected object");
        }

        const auto ref = IO::readFitFileReference(*yc_it,
                                                  base_dir,
                                                  Detail::fieldContext(entity, "yc"));
        auto       fit = IO::readCharacteristicAdmittanceFit<RealT>(
            ref,
            Detail::fieldContext(entity, "yc.fit_file"));

        if (fit.phase_order != data.phase_order)
        {
          throw CaseError(std::string(Detail::fieldContext(entity, "phase_order"))
                          + ": does not match Yc fit phase_order");
        }

        data.yc_fit_file = fit.path;
        data.yc_fit_name = std::move(fit.fit_name);
        data.yc          = std::move(fit.data);
        return BranchFrequencyDependent(std::move(data));
      }

      const Data& data() const
      {
        return data_;
      }

      size_t variableCount() const
      {
        return 2u * stateCount();
      }

      size_t equationCount() const
      {
        return variableCount();
      }

      bool differential(size_t local) const
      {
        return local < variableCount();
      }

      size_t stateCount() const
      {
        return yc_from_.stateCount();
      }

      template <class Init>
      void initialize(Init& init) const
      {
        const auto n = stateCount();
        if (n == 0u)
        {
          return;
        }

        const auto v_from  = init.voltage(static_cast<IdxT>(from));
        const auto vp_from = init.voltageDerivative(static_cast<IdxT>(from));
        const auto v_to    = init.voltage(static_cast<IdxT>(to));
        const auto vp_to   = init.voltageDerivative(static_cast<IdxT>(to));

        yc_from_.initialize(init.y(IdxT{0}, static_cast<IdxT>(n)),
                            init.yp(IdxT{0}, static_cast<IdxT>(n)),
                            v_from,
                            vp_from);
        yc_to_.initialize(init.y(static_cast<IdxT>(n), static_cast<IdxT>(n)),
                          init.yp(static_cast<IdxT>(n), static_cast<IdxT>(n)),
                          v_to,
                          vp_to);
      }

      template <class Variables, class Residual>
      void residual(const Variables& x, Residual& eq) const
      {
        const auto n = stateCount();

        const auto injection_from = portInjectionWithResidual(
            yc_from_,
            x.y(IdxT{0}, static_cast<IdxT>(n)),
            x.yp(IdxT{0}, static_cast<IdxT>(n)),
            eq.f(IdxT{0}, static_cast<IdxT>(n)),
            x.voltage(static_cast<IdxT>(from)),
            x.voltageDerivative(static_cast<IdxT>(from)));

        const auto injection_to = portInjectionWithResidual(
            yc_to_,
            x.y(static_cast<IdxT>(n), static_cast<IdxT>(n)),
            x.yp(static_cast<IdxT>(n), static_cast<IdxT>(n)),
            eq.f(static_cast<IdxT>(n), static_cast<IdxT>(n)),
            x.voltage(static_cast<IdxT>(to)),
            x.voltageDerivative(static_cast<IdxT>(to)));

        eq.injectCurrent(static_cast<IdxT>(from), injection_from);
        eq.injectCurrent(static_cast<IdxT>(to), injection_to);
      }

      template <class Scalar>
      PhaseVector<Scalar> portInjection(size_t                 port,
                                        std::span<const Scalar> y,
                                        std::span<const Scalar> yp,
                                        const PhaseVector<Scalar>& voltage,
                                        const PhaseVector<Scalar>& voltage_derivative) const
      {
        if (port != from && port != to)
        {
          throw std::invalid_argument("EMT BranchFrequencyDependent port index is out of range");
        }
        std::vector<Scalar> scratch(stateCount(), Scalar{0.0});
        return portInjectionWithResidual(port == from ? yc_from_ : yc_to_,
                                         y,
                                         yp,
                                         std::span<Scalar>{scratch.data(), scratch.size()},
                                         voltage,
                                         voltage_derivative);
      }

    private:
      static void requirePhaseOrder(const std::vector<std::string>& phases,
                                    std::string_view                entity)
      {
        if (phases != std::vector<std::string>{"a", "b", "c"})
        {
          throw CaseError(std::string(entity) + ": expected ['a','b','c']");
        }
      }

      template <class Scalar>
      PhaseVector<Scalar> portInjectionWithResidual(const Math::RationalApprox<RealT>& model,
                                                    std::span<const Scalar>            y,
                                                    std::span<const Scalar>            yp,
                                                    std::span<Scalar>                  f,
                                                    const PhaseVector<Scalar>&         voltage,
                                                    const PhaseVector<Scalar>&         voltage_derivative) const
      {
        PhaseVector<Scalar> current{};
        model.residual(y, yp, f, voltage, voltage_derivative, current);
        for (auto& value : current)
        {
          value = -value;
        }
        return current;
      }

      Data                         data_;
      Math::RationalApprox<RealT>  yc_from_;
      Math::RationalApprox<RealT>  yc_to_;
    };

    enum class BranchFrequencyDependentMonitorVariable
    {
      ifa,
      ifb,
      ifc,
      ita,
      itb,
      itc
    };

    template <class RealT, class IdxT>
    struct ComponentDescriptor<BranchFrequencyDependent<RealT, IdxT>>
    {
      using Component = BranchFrequencyDependent<RealT, IdxT>;
      using Data      = BranchFrequencyDependentData<RealT, IdxT>;

      static constexpr std::string_view                class_name = "BranchFrequencyDependent";
      static constexpr std::array<std::string_view, 2> electrical_ports{"from", "to"};
      static constexpr std::array<std::string_view, 0> input_ports{};
      static constexpr std::array<std::string_view, 0> output_ports{};
      static constexpr auto                            params = std::tuple{};

      static_assert(electrical_ports.size() == ComponentTraits<Component>::electrical_port_count);
      static_assert(input_ports.size() == ComponentTraits<Component>::input_port_count);
      static_assert(output_ports.size() == ComponentTraits<Component>::output_port_count);
    };

    template <class RealT, class IdxT>
    struct ComponentMonitorTraits<BranchFrequencyDependent<RealT, IdxT>>
    {
      using Variable = BranchFrequencyDependentMonitorVariable;

      static std::optional<Variable> resolve(std::string_view name)
      {
        return resolveMonitorVariable<Variable>(name);
      }

      template <class Context>
      static void bind(Context&                                      ctx,
                       const BranchFrequencyDependent<RealT, IdxT>& component,
                       Variable                                     variable)
      {
        const auto [label, port, phase] = monitorSpec(variable);
        const auto state_count          = component.stateCount();
        const auto local_state_offset   = port == BranchFrequencyDependent<RealT, IdxT>::from
                                            ? 0u
                                            : state_count;
        const auto global_state_offset  = state_count == 0u
                                            ? typename Context::index_type{0}
                                            : ctx.componentVariableIndex(local_state_offset);
        const auto y                    = ctx.y();
        const auto yp                   = ctx.yp();
        const std::array<typename Context::index_type, 3> voltage_indices{
            ctx.portVoltageIndex(port, 0),
            ctx.portVoltageIndex(port, 1),
            ctx.portVoltageIndex(port, 2)};
        const std::array<typename Context::index_type, 3> derivative_indices{
            ctx.portVoltageDerivativeIndex(port, 0),
            ctx.portVoltageDerivativeIndex(port, 1),
            ctx.portVoltageDerivativeIndex(port, 2)};

        ctx.add(std::string(label),
                [&component,
                 port,
                 phase,
                 state_count,
                 global_state_offset,
                 y,
                 yp,
                 voltage_indices,
                 derivative_indices]()
                {
                  using Scalar = typename Context::scalar_type;
                  PhaseVector<Scalar> voltage{};
                  PhaseVector<Scalar> voltage_derivative{};
                  for (std::size_t i = 0; i < 3u; ++i)
                  {
                    voltage[i] = (*y)[static_cast<std::size_t>(voltage_indices[i])];
                    voltage_derivative[i] =
                        (*yp)[static_cast<std::size_t>(derivative_indices[i])];
                  }

                  std::span<const Scalar> state_y{};
                  std::span<const Scalar> state_yp{};
                  if (state_count > 0u)
                  {
                    state_y = std::span<const Scalar>{
                        y->data() + static_cast<std::size_t>(global_state_offset),
                        state_count};
                    state_yp = std::span<const Scalar>{
                        yp->data() + static_cast<std::size_t>(global_state_offset),
                        state_count};
                  }
                  return component.portInjection(port,
                                                 state_y,
                                                 state_yp,
                                                 voltage,
                                                 voltage_derivative)[phase];
                });
      }

    private:
      static std::tuple<std::string_view, std::size_t, std::size_t> monitorSpec(Variable variable)
      {
        using Component = BranchFrequencyDependent<RealT, IdxT>;
        switch (variable)
        {
        case Variable::ifa:
          return {"ifa", Component::from, 0u};
        case Variable::ifb:
          return {"ifb", Component::from, 1u};
        case Variable::ifc:
          return {"ifc", Component::from, 2u};
        case Variable::ita:
          return {"ita", Component::to, 0u};
        case Variable::itb:
          return {"itb", Component::to, 1u};
        case Variable::itc:
          return {"itc", Component::to, 2u};
        }
        throw std::invalid_argument("Invalid EMT BranchFrequencyDependent monitor variable");
      }
    };

    template <class RealT, class IdxT>
    struct ComponentJacobianTraits<BranchFrequencyDependent<RealT, IdxT>>
    {
      static constexpr ComponentJacobianForm              form = ComponentJacobianForm::Affine;
      static constexpr ComponentJacobianCoefficientUpdate coefficient_update =
          ComponentJacobianCoefficientUpdate::Static;
    };
  } // namespace EMT
} // namespace GridKit
