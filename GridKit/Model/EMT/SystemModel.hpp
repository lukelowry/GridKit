#pragma once

#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/System/Assembly.hpp>
#include <GridKit/Model/EMT/System/ComponentTraits.hpp>
#include <GridKit/Model/EMT/System/EnzymeJacobian.hpp>
#include <GridKit/Model/EMT/System/JacobianView.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>
#include <GridKit/Model/EMT/System/NetworkData.hpp>
#include <GridKit/Model/EMT/System/PatternView.hpp>
#include <GridKit/Model/EMT/System/ResidualView.hpp>
#include <GridKit/Model/EMT/System/VariableView.hpp>
#include <GridKit/Model/Evaluator.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class NetworkT>
    class SystemModel
      : public GridKit::Model::Evaluator<typename NetworkT::scalar_type, typename NetworkT::index_type>
    {
    public:
      using ScalarT    = typename NetworkT::scalar_type;
      using IdxT       = typename NetworkT::index_type;
      using StoreT     = typename NetworkT::component_store_type;
      using Base       = GridKit::Model::Evaluator<ScalarT, IdxT>;
      using RealT      = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using MatrixT    = typename Base::MatrixT;
      using CsrMatrixT = typename Base::CsrMatrixT;

      explicit SystemModel(NetworkT network,
                           RealT    rel_tol   = 1e-4,
                           RealT    abs_tol   = 1e-4,
                           bool     use_jac   = true,
                           IdxT     max_steps = 2000)
        : network_{std::move(network)},
          rel_tol_{rel_tol},
          abs_tol_{abs_tol},
          use_jac_{use_jac},
          max_steps_{max_steps}
      {
      }

      int allocate() override
      {
        allocated_ = false;
        buildLayout();
        if (layout_.size() != layout_.equations())
        {
          throw std::invalid_argument("EMT layout must have one residual equation per variable");
        }

        y_.assign(layout_.size(), ScalarT{0.0});
        yp_.assign(layout_.size(), ScalarT{0.0});
        f_.assign(layout_.size(), ScalarT{0.0});
        tag_.assign(layout_.size(), false);
        differential_variables_.assign(layout_.size(), false);

        prepareResolvedStorage();
        bindTerminalConnections();
        validateTerminals();
        bindPortConnections();
        validateInputs();

        markStaticDifferentialVariables();
        buildAssembly();
        allocated_ = true;
        return 0;
      }

      int initialize() override
      {
        ensureAllocated();
        std::fill(y_.begin(), y_.end(), ScalarT{0.0});
        std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
        std::fill(f_.begin(), f_.end(), ScalarT{0.0});

        for (IdxT bus = 0; bus < static_cast<IdxT>(network_.buses.size()); ++bus)
        {
          network_.buses[bus].initialize(y_.data(), yp_.data(), layout_.busVariable(bus, 0));
        }
        return 0;
      }

      int tagDifferentiable() override
      {
        ensureAllocated();
        tag_ = differential_variables_;
        return 0;
      }

      int evaluateResidual() override
      {
        ensureAllocated();
        std::fill(f_.begin(), f_.end(), ScalarT{0.0});
        network_.components.forEach(
            [&](auto& component, ComponentId id)
            {
              const auto& slot               = layout_.component(id);
              auto        terminal_bus_ids   = layout_.terminals(resolved_terminal_bus_ids_, id);
              auto        input_variable_ids = layout_.inputs(resolved_input_variable_ids_, id);

              VariableView<ScalarT, IdxT> variables(y_.data(),
                                                    yp_.data(),
                                                    layout_,
                                                    slot,
                                                    terminal_bus_ids,
                                                    input_variable_ids);
              ResidualView<ScalarT, IdxT> residual(f_.data(), layout_, slot, terminal_bus_ids);
              component.residual(variables, residual);
            });
        return 0;
      }

      int evaluateJacobian() override
      {
        ensureAllocated();
        assembly_.clearValues();
        network_.components.forEach(
            [&](auto& component, ComponentId id)
            {
              using ComponentT               = std::decay_t<decltype(component)>;
              const auto& slot               = layout_.component(id);
              auto        terminal_bus_ids   = layout_.terminals(resolved_terminal_bus_ids_, id);
              auto        input_variable_ids = layout_.inputs(resolved_input_variable_ids_, id);

              VariableView<ScalarT, IdxT> variables(y_.data(),
                                                    yp_.data(),
                                                    layout_,
                                                    slot,
                                                    terminal_bus_ids,
                                                    input_variable_ids);
              JacobianView<RealT, IdxT>   jacobian(assembly_.values(),
                                                 assembly_.rowPtrs(),
                                                 assembly_.columns(),
                                                 layout_,
                                                 slot,
                                                 terminal_bus_ids,
                                                 input_variable_ids,
                                                 alpha_);

              if constexpr (ComponentTraits<ComponentT>::direct_jacobian)
              {
                component.jacobian(variables, jacobian);
              }
              else
              {
                EnzymeJacobian<ComponentT>::evaluate(component, variables, jacobian);
              }
            });
        return 0;
      }

      int evaluateIntegrand() override
      {
        return 0;
      }

      int initializeAdjoint() override
      {
        return 0;
      }

      int evaluateAdjointResidual() override
      {
        return 0;
      }

      int evaluateAdjointIntegrand() override
      {
        return 0;
      }

      IdxT size() override
      {
        return layout_.size();
      }

      IdxT nnz() override
      {
        return assembly_.nnz();
      }

      CsrMatrixT* getCsrJacobian() const override
      {
        return assembly_.matrix();
      }

      bool hasJacobian() override
      {
        return use_jac_ && jacobianAvailable();
      }

      IdxT sizeQuadrature() override
      {
        return static_cast<IdxT>(integrand_.size());
      }

      IdxT sizeParams() override
      {
        return static_cast<IdxT>(param_.size());
      }

      void updateTime(RealT t, RealT a) override
      {
        time_  = t;
        alpha_ = a;
      }

      void setTolerances(RealT& rtol, RealT& atol) const override
      {
        rtol = rel_tol_;
        atol = abs_tol_;
      }

      void setMaxSteps(IdxT& msa) const override
      {
        msa = max_steps_;
      }

      std::vector<ScalarT>& y() override
      {
        return y_;
      }

      const std::vector<ScalarT>& y() const override
      {
        return y_;
      }

      std::vector<ScalarT>& yp() override
      {
        return yp_;
      }

      const std::vector<ScalarT>& yp() const override
      {
        return yp_;
      }

      std::vector<bool>& tag() override
      {
        return tag_;
      }

      const std::vector<bool>& tag() const override
      {
        return tag_;
      }

      std::vector<ScalarT>& yB() override
      {
        return yB_;
      }

      const std::vector<ScalarT>& yB() const override
      {
        return yB_;
      }

      std::vector<ScalarT>& ypB() override
      {
        return ypB_;
      }

      const std::vector<ScalarT>& ypB() const override
      {
        return ypB_;
      }

      std::vector<ScalarT>& param() override
      {
        return param_;
      }

      const std::vector<ScalarT>& param() const override
      {
        return param_;
      }

      std::vector<ScalarT>& param_up() override
      {
        return param_up_;
      }

      const std::vector<ScalarT>& param_up() const override
      {
        return param_up_;
      }

      std::vector<ScalarT>& param_lo() override
      {
        return param_lo_;
      }

      const std::vector<ScalarT>& param_lo() const override
      {
        return param_lo_;
      }

      std::vector<ScalarT>& getResidual() override
      {
        return f_;
      }

      const std::vector<ScalarT>& getResidual() const override
      {
        return f_;
      }

      MatrixT& getJacobian() override
      {
        return jacobian_;
      }

      const MatrixT& getJacobian() const override
      {
        return jacobian_;
      }

      std::vector<ScalarT>& getIntegrand() override
      {
        return integrand_;
      }

      const std::vector<ScalarT>& getIntegrand() const override
      {
        return integrand_;
      }

      std::vector<ScalarT>& getAdjointResidual() override
      {
        return adjoint_residual_;
      }

      const std::vector<ScalarT>& getAdjointResidual() const override
      {
        return adjoint_residual_;
      }

      std::vector<ScalarT>& getAdjointIntegrand() override
      {
        return adjoint_integrand_;
      }

      const std::vector<ScalarT>& getAdjointIntegrand() const override
      {
        return adjoint_integrand_;
      }

      const Layout<IdxT>& layout() const
      {
        return layout_;
      }

      const Assembly<RealT, IdxT>& assembly() const
      {
        return assembly_;
      }

      NetworkT& network()
      {
        return network_;
      }

      const NetworkT& network() const
      {
        return network_;
      }

    private:
      void ensureAllocated()
      {
        if (!allocated_)
        {
          allocate();
        }
      }

      void buildLayout()
      {
        layout_.reset(static_cast<IdxT>(network_.buses.size()));
        layout_.setComponentTypes(StoreT::typeCount());

        network_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              layout_.appendComponent(id.type,
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::variables),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::equations),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::terminals),
                                      static_cast<IdxT>(ComponentTraits<ComponentT>::inputs));
            });
      }

      void prepareResolvedStorage()
      {
        resolved_terminal_bus_ids_.assign(layout_.terminalCount(), INVALID_INDEX<IdxT>);
        resolved_input_variable_ids_.assign(layout_.inputCount(), INVALID_INDEX<IdxT>);
      }

      void markStaticDifferentialVariables()
      {
        network_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              const auto& slot = layout_.component(id);
              for (IdxT local = 0; local < slot.variable_count; ++local)
              {
                differential_variables_[slot.variable_offset + local] =
                    differential_variables_[slot.variable_offset + local]
                    || ComponentTraits<ComponentT>::differential(local);
              }
            });
      }

      void bindTerminalConnections()
      {
        for (const auto& connection : network_.terminal_connections)
        {
          if (connection.bus >= static_cast<IdxT>(network_.buses.size()))
          {
            throw std::invalid_argument("EMT terminal connection references a bus that does not exist");
          }

          const bool found = network_.components.visit(
              connection.terminal.component,
              [&](const auto& component)
              {
                using ComponentT = std::decay_t<decltype(component)>;
                (void) component;
                if (connection.terminal.index >= ComponentTraits<ComponentT>::terminals)
                {
                  throw std::invalid_argument("EMT terminal index is out of range");
                }

                const auto& slot   = layout_.component(connection.terminal.component);
                IdxT&       bus_id = resolved_terminal_bus_ids_[slot.terminal_offset + connection.terminal.index];
                if (bus_id != INVALID_INDEX<IdxT>)
                {
                  throw std::invalid_argument("EMT terminal is connected more than once");
                }
                bus_id = connection.bus;
              });

          if (!found)
          {
            throw std::invalid_argument("EMT terminal connection component does not exist");
          }
        }
      }

      void validateTerminals() const
      {
        network_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              (void) component;
              const auto& slot = layout_.component(id);
              for (IdxT local = 0; local < slot.terminal_count; ++local)
              {
                if (resolved_terminal_bus_ids_[slot.terminal_offset + local] == INVALID_INDEX<IdxT>)
                {
                  throw std::invalid_argument("EMT component terminal is not connected");
                }
              }
            });
      }

      void bindPortConnections()
      {
        for (const auto& connection : network_.port_connections)
        {
          const IdxT output_variable = resolveOutputVariable(connection.output);
          bindInput(connection.input, output_variable);
        }
      }

      IdxT resolveOutputVariable(const OutputRef& output) const
      {
        IdxT       variable = INVALID_INDEX<IdxT>;
        const bool found    = network_.components.visit(
            output.component,
            [&](const auto& component)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              if (output.index >= ComponentTraits<ComponentT>::outputs)
              {
                throw std::invalid_argument("EMT output index is out of range");
              }

              const OutputSpec spec = ComponentTraits<ComponentT>::output(output.index);
              if (spec.variable >= ComponentTraits<ComponentT>::variables)
              {
                throw std::invalid_argument("EMT output variable is out of range");
              }
              variable = layout_.component(output.component).variable_offset + spec.variable;
            });

        if (!found)
        {
          throw std::invalid_argument("EMT output component does not exist");
        }
        return variable;
      }

      void bindInput(const InputRef& input, IdxT output_variable)
      {
        const bool found = network_.components.visit(
            input.component,
            [&](const auto& component)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              if (input.index >= ComponentTraits<ComponentT>::inputs)
              {
                throw std::invalid_argument("EMT input index is out of range");
              }

              const auto& slot           = layout_.component(input.component);
              IdxT&       input_variable = resolved_input_variable_ids_[slot.input_offset + input.index];
              if (input_variable != INVALID_INDEX<IdxT>)
              {
                throw std::invalid_argument("EMT input is connected more than once");
              }
              input_variable = output_variable;
            });

        if (!found)
        {
          throw std::invalid_argument("EMT input component does not exist");
        }
      }

      void validateInputs() const
      {
        network_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              (void) component;
              const auto& slot = layout_.component(id);
              for (IdxT local = 0; local < slot.input_count; ++local)
              {
                if (resolved_input_variable_ids_[slot.input_offset + local] == INVALID_INDEX<IdxT>)
                {
                  throw std::invalid_argument("EMT input is not connected");
                }
              }
            });
      }

      void buildAssembly()
      {
        std::vector<typename Assembly<RealT, IdxT>::Entry> entries;
        network_.components.forEach(
            [&](const auto& component, ComponentId id)
            {
              using ComponentT                            = std::decay_t<decltype(component)>;
              const auto&              slot               = layout_.component(id);
              auto                     terminal_bus_ids   = layout_.terminals(resolved_terminal_bus_ids_, id);
              auto                     input_variable_ids = layout_.inputs(resolved_input_variable_ids_, id);
              PatternView<RealT, IdxT> pattern(entries,
                                               layout_,
                                               slot,
                                               terminal_bus_ids,
                                               input_variable_ids,
                                               &differential_variables_);

              if constexpr (requires(PatternView<RealT, IdxT>& p) { component.pattern(p); })
              {
                component.pattern(pattern);
              }
              else
              {
                EnzymeJacobian<ComponentT>::pattern(component, pattern);
              }
            });
        assembly_.build(layout_.size(), std::move(entries));
      }

      bool jacobianAvailable() const
      {
        bool available = true;
        network_.components.forEach(
            [&](const auto& component, ComponentId)
            {
              using ComponentT = std::decay_t<decltype(component)>;
              (void) component;
              if constexpr (!ComponentTraits<ComponentT>::direct_jacobian)
              {
#ifndef GRIDKIT_ENABLE_ENZYME
                available = false;
#endif
              }
            });
        return available;
      }

      NetworkT              network_;
      Layout<IdxT>          layout_;
      Assembly<RealT, IdxT> assembly_;

      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<ScalarT> f_;
      std::vector<bool>    tag_;
      std::vector<bool>    differential_variables_;

      std::vector<ScalarT> yB_;
      std::vector<ScalarT> ypB_;
      std::vector<ScalarT> param_;
      std::vector<ScalarT> param_up_;
      std::vector<ScalarT> param_lo_;
      std::vector<ScalarT> integrand_;
      std::vector<ScalarT> adjoint_residual_;
      std::vector<ScalarT> adjoint_integrand_;

      MatrixT jacobian_;

      std::vector<IdxT> resolved_terminal_bus_ids_;
      std::vector<IdxT> resolved_input_variable_ids_;

      RealT rel_tol_{1e-4};
      RealT abs_tol_{1e-4};
      RealT time_{0.0};
      RealT alpha_{0.0};
      bool  use_jac_{true};
      bool  allocated_{false};
      IdxT  max_steps_{2000};
    };
  } // namespace EMT
} // namespace GridKit
