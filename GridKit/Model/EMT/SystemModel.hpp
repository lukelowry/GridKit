#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <GridKit/Definitions.hpp>
#include <GridKit/LinearAlgebra/MemoryUtils.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/System/CsrAssembly.hpp>
#include <GridKit/Model/EMT/System/Layout.hpp>
#include <GridKit/Model/EMT/System/LocalMap.hpp>
#include <GridKit/Model/EMT/System/SparseAD.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/VariableMonitorController.hpp>
#include <GridKit/Model/VariableMonitorImpl.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class SystemModel : public EMT::Component<ScalarT, IdxT>
    {
      using bus_type           = EMT::Bus<ScalarT, IdxT>;
      using component_type     = EMT::Component<ScalarT, IdxT>;
      using branch_type        = EMT::BranchLumpedConstant<ScalarT, IdxT>;
      using load_type          = EMT::LoadRL<ScalarT, IdxT>;
      using source_type        = EMT::VoltageSource<ScalarT, IdxT>;
      using breaker_type       = EMT::Breaker<ScalarT, IdxT>;
      using RealT              = typename Model::Evaluator<ScalarT, IdxT>::RealT;
      using CsrMatrixT         = typename Model::Evaluator<ScalarT, IdxT>::CsrMatrixT;
      using LocalMapT          = EMT::System::LocalMap<IdxT>;
      using ComponentBlockT    = EMT::System::ComponentBlock<IdxT>;
      using ComponentJacobianT = EMT::System::ComponentJacobian<IdxT>;

      using EMT::Component<ScalarT, IdxT>::gridkit_component_id_;
      using EMT::Component<ScalarT, IdxT>::size_;
      using EMT::Component<ScalarT, IdxT>::nnz_;
      using EMT::Component<ScalarT, IdxT>::time_;
      using EMT::Component<ScalarT, IdxT>::alpha_;
      using EMT::Component<ScalarT, IdxT>::y_;
      using EMT::Component<ScalarT, IdxT>::yp_;
      using EMT::Component<ScalarT, IdxT>::tag_;
      using EMT::Component<ScalarT, IdxT>::f_;
      using EMT::Component<ScalarT, IdxT>::variable_indices_;
      using EMT::Component<ScalarT, IdxT>::residual_indices_;

      struct RuntimeComponent
      {
        EMT::System::ComponentKind kind;
        std::size_t                index{0};
        ComponentBlockT            block;
        ComponentJacobianT         jacobian;
      };

    public:
      SystemModel()
        : monitor_(time_)
      {
      }

      explicit SystemModel(const SystemModelData<RealT, IdxT>& data)
        : monitor_(time_)
      {
        owns_components_ = true;

        for (const auto& bus_data : data.bus)
        {
          addBus(new Bus<ScalarT, IdxT>(bus_data));
        }

        for (const auto& component_data : data.branch_lumped_constant)
        {
          addComponent(new BranchLumpedConstant<ScalarT, IdxT>(
              getBus(component_data.ports.at(BranchLumpedConstantData<RealT, IdxT>::Ports::from)),
              getBus(component_data.ports.at(BranchLumpedConstantData<RealT, IdxT>::Ports::to)),
              component_data));
        }

        for (const auto& component_data : data.load_rl)
        {
          addComponent(new LoadRL<ScalarT, IdxT>(
              getBus(component_data.ports.at(LoadRLData<RealT, IdxT>::Ports::ac)),
              component_data));
        }

        for (const auto& component_data : data.voltage_source)
        {
          addComponent(new VoltageSource<ScalarT, IdxT>(
              getBus(component_data.ports.at(VoltageSourceData<RealT, IdxT>::Ports::bus)),
              component_data));
        }

        for (const auto& component_data : data.breaker)
        {
          addComponent(new Breaker<ScalarT, IdxT>(
              getBus(component_data.ports.at(BreakerData<RealT, IdxT>::Ports::from)),
              getBus(component_data.ports.at(BreakerData<RealT, IdxT>::Ports::to)),
              component_data));
        }

        for (const auto& sink : data.monitor_sink)
        {
          monitor_.addSink(sink);
        }
      }

      virtual ~SystemModel()
      {
        if (owns_components_)
        {
          for (auto* component : all_components_)
          {
            delete component;
          }

          for (auto* bus : buses_)
          {
            delete bus;
          }
        }
      }

      int setGridKitComponentID(IdxT component_id) override
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      int allocate() override
      {
        layout_ = EMT::System::Layout<RealT, IdxT>{};
        runtime_components_.clear();
        derivative_columns_.clear();
        csr_rows_.clear();
        csr_cols_.clear();
        csr_values_.clear();
        monitor_bindings_.clear();
        csr_jac_.reset();

        size_ = 0;
        for (auto* bus : buses_)
        {
          bus->allocate();
          for (IdxT phase = 0; phase < bus_type::variable_count; ++phase)
          {
            bus->setVariableIndex(phase, size_ + phase);
            bus->setResidualIndex(phase, size_ + phase);
          }

          typename EMT::System::BusBlock<RealT, IdxT> block;
          block.external_id    = bus->busID();
          block.name           = bus->name();
          block.variable_start = size_;
          block.equation_start = size_;
          block.vm             = bus->data().vm;
          block.va             = bus->data().va;
          block.frequency      = bus->frequency();
          layout_.buses.push_back(block);

          size_ += bus_type::variable_count;
        }

        addRuntimeBlocks<branch_type>(EMT::System::ComponentKind::BranchLumpedConstant, branches_);
        addRuntimeBlocks<load_type>(EMT::System::ComponentKind::LoadRL, loads_);
        addRuntimeBlocks<source_type>(EMT::System::ComponentKind::VoltageSource, sources_);

        layout_.size = size_;

        y_.assign(static_cast<std::size_t>(size_), ScalarT{0.0});
        yp_.assign(static_cast<std::size_t>(size_), ScalarT{0.0});
        f_.assign(static_cast<std::size_t>(size_), ScalarT{0.0});
        tag_.assign(static_cast<std::size_t>(size_), false);
        variable_indices_.resize(static_cast<std::size_t>(size_));
        residual_indices_.resize(static_cast<std::size_t>(size_));

        for (IdxT idx = 0; idx < size_; ++idx)
        {
          this->setVariableIndex(idx, idx);
          this->setResidualIndex(idx, idx);
        }

        const int error_count = this->verify();
        if (error_count > 0)
        {
          Log::error() << "EMT component errors: " << error_count << std::endl;
          throw std::runtime_error("EMT SystemModel allocation failed");
        }

        buildSparseJacobianPattern();
        tagDifferentiable();
        initializeMonitor();
        startMonitor();
        return 0;
      }

      int verify() const override
      {
        int ret = 0;
        for (const auto* bus : buses_)
        {
          ret += bus->verify();
        }
        for (const auto* component : all_components_)
        {
          ret += component->verify();
        }
        return ret;
      }

      bool hasJacobian() override
      {
        return true;
      }

      int initialize() override
      {
        for (std::size_t bus_index = 0; bus_index < buses_.size(); ++bus_index)
        {
          auto* bus = buses_[bus_index];
          bus->initialize();
          const auto& block = layout_.buses.at(bus_index);
          for (IdxT phase = 0; phase < bus_type::variable_count; ++phase)
          {
            const auto global = static_cast<std::size_t>(block.variable_start + phase);
            y_[global]        = bus->y().at(static_cast<std::size_t>(phase));
            yp_[global]       = bus->yp().at(static_cast<std::size_t>(phase));
          }
        }

        for (auto* branch : branches_)
        {
          branch->initialize();
          copyOwnStateToGlobal(*branch);
        }

        for (auto* load : loads_)
        {
          load->initialize();
          copyOwnStateToGlobal(*load);
        }

        for (auto* source : sources_)
        {
          source->initialize();
        }

        return 0;
      }

      void initializeMonitor()
      {
        for (auto* bus : buses_)
        {
          bindBusMonitor(*bus);
        }

        for (auto* branch : branches_)
        {
          bindBranchMonitor(*branch);
        }
        for (auto* load : loads_)
        {
          bindLoadMonitor(*load);
        }
        for (auto* source : sources_)
        {
          bindVoltageSourceMonitor(*source);
        }
      }

      void startMonitor() override
      {
        monitor_.start();
      }

      void stopMonitor() override
      {
        monitor_.stop();
      }

      bool monitoring() const override
      {
        return !monitor_.empty();
      }

      void printMonitoredVariables() const override
      {
        monitor_.print();
      }

      int tagDifferentiable() override
      {
        // TODO: re-run on structural change (Breaker). The current derivative column set is built once in allocate(),
        // which is correct for the static affine EMT models but not for runtime equation reclassification.
        std::fill(tag_.begin(), tag_.end(), false);
        for (const auto column : derivative_columns_)
        {
          tag_.at(static_cast<std::size_t>(column)) = true;
        }
        return 0;
      }

      int evaluateResidual() override
      {
        std::fill(f_.begin(), f_.end(), ScalarT{0.0});

        for (const auto& runtime : runtime_components_)
        {
          const auto& map = runtime.block.local_map;
          map.gather(y_, yp_, local_y_, local_yp_);
          local_f_.assign(map.localEquationCount(), ScalarT{0.0});

          switch (runtime.kind)
          {
          case EMT::System::ComponentKind::BranchLumpedConstant:
            branches_.at(runtime.index)->residual(local_y_.data(), local_yp_.data(), local_f_.data(), time_);
            break;
          case EMT::System::ComponentKind::LoadRL:
            loads_.at(runtime.index)->residual(local_y_.data(), local_yp_.data(), local_f_.data(), time_);
            break;
          case EMT::System::ComponentKind::VoltageSource:
            sources_.at(runtime.index)->residual(local_y_.data(), local_yp_.data(), local_f_.data(), time_);
            break;
          }

          map.scatterResidual(local_f_, f_);
        }

        for (const auto* bus : buses_)
        {
          bus->addIntrinsicResidual(y_, yp_, f_);
        }

        return 0;
      }

      int evaluateJacobian() override
      {
        std::fill(csr_values_.begin(), csr_values_.end(), RealT{0.0});

        for (const auto& runtime : runtime_components_)
        {
          switch (runtime.kind)
          {
          case EMT::System::ComponentKind::BranchLumpedConstant:
            EMT::System::SparseAD<IdxT>::addComponentJacobian(*branches_.at(runtime.index),
                                                              runtime.jacobian,
                                                              runtime.block.local_map,
                                                              y_,
                                                              yp_,
                                                              alpha_,
                                                              csr_values_,
                                                              time_,
                                                              jac_local_y_,
                                                              jac_local_yp_,
                                                              jac_work_f_);
            break;
          case EMT::System::ComponentKind::LoadRL:
            EMT::System::SparseAD<IdxT>::addComponentJacobian(*loads_.at(runtime.index),
                                                              runtime.jacobian,
                                                              runtime.block.local_map,
                                                              y_,
                                                              yp_,
                                                              alpha_,
                                                              csr_values_,
                                                              time_,
                                                              jac_local_y_,
                                                              jac_local_yp_,
                                                              jac_work_f_);
            break;
          case EMT::System::ComponentKind::VoltageSource:
            EMT::System::SparseAD<IdxT>::addComponentJacobian(*sources_.at(runtime.index),
                                                              runtime.jacobian,
                                                              runtime.block.local_map,
                                                              y_,
                                                              yp_,
                                                              alpha_,
                                                              csr_values_,
                                                              time_,
                                                              jac_local_y_,
                                                              jac_local_yp_,
                                                              jac_work_f_);
            break;
          }
        }

        for (const auto* bus : buses_)
        {
          bus->addIntrinsicJacobian(layout_, y_, yp_, alpha_, csr_values_);
        }

        if (csr_jac_ != nullptr)
        {
          csr_jac_->setUpdated(LinearAlgebra::memory::HOST);
        }
        return 0;
      }

      CsrMatrixT* getCsrJacobian() const override
      {
        return csr_jac_.get();
      }

      void updateVariables()
      {
      }

      void updateTime(RealT t, RealT a) override
      {
        time_  = t;
        alpha_ = a;
        for (auto* component : all_components_)
        {
          component->updateTime(t, a);
        }
      }

      void addBus(bus_type* bus)
      {
        const IdxT gridkit_bus_id = static_cast<IdxT>(buses_.size());
        if (!gridkit_bus_indices_.contains(bus->busID()))
        {
          gridkit_bus_indices_[bus->busID()] = gridkit_bus_id;
        }
        else
        {
          bus->setBusID(gridkit_bus_id);
          gridkit_bus_indices_[bus->busID()] = gridkit_bus_id;
        }
        if (!bus->name().empty())
        {
          bus_name_to_index_[bus->name()] = gridkit_bus_id;
        }
        buses_.push_back(bus);
      }

      void addComponent(component_type* component)
      {
        const IdxT gridkit_component_id = static_cast<IdxT>(all_components_.size());
        component->setGridKitComponentID(gridkit_component_id);
        all_components_.push_back(component);

        if (auto* branch = dynamic_cast<branch_type*>(component))
        {
          branches_.push_back(branch);
        }
        else if (auto* load = dynamic_cast<load_type*>(component))
        {
          loads_.push_back(load);
        }
        else if (auto* source = dynamic_cast<source_type*>(component))
        {
          sources_.push_back(source);
        }
      }

      bus_type* getBus(IdxT bus_id)
      {
        const IdxT gridkit_bus_id = gridkit_bus_indices_.at(bus_id);
        assert((buses_[static_cast<std::size_t>(gridkit_bus_id)])->busID() == bus_id);
        return buses_[static_cast<std::size_t>(gridkit_bus_id)];
      }

      bus_type* getBusByName(const std::string& name)
      {
        return buses_[static_cast<std::size_t>(bus_name_to_index_.at(name))];
      }

      component_type* getComponent(IdxT gridkit_component_id)
      {
        return all_components_.at(static_cast<std::size_t>(gridkit_component_id));
      }

      const EMT::System::Layout<RealT, IdxT>& layout() const
      {
        return layout_;
      }

    private:
      template <class ComponentT>
      void addRuntimeBlocks(EMT::System::ComponentKind kind, const std::vector<ComponentT*>& components)
      {
        for (std::size_t component_index = 0; component_index < components.size(); ++component_index)
        {
          auto* component = components[component_index];
          component->allocate();

          const IdxT own_variable_start = size_;
          for (std::size_t local = 0; local < ComponentT::own_variable_count; ++local)
          {
            component->setVariableIndex(static_cast<IdxT>(local),
                                        own_variable_start + static_cast<IdxT>(local));
            component->setResidualIndex(static_cast<IdxT>(local),
                                        own_variable_start + static_cast<IdxT>(local));
          }
          size_ += static_cast<IdxT>(ComponentT::own_variable_count);

          RuntimeComponent runtime;
          runtime.kind                     = kind;
          runtime.index                    = component_index;
          runtime.block.kind               = kind;
          runtime.block.component_index    = static_cast<IdxT>(component_index);
          runtime.block.own_variable_start = own_variable_start;
          runtime.block.own_equation_start = own_variable_start;
          runtime.block.local_map          = makeLocalMap(*component, own_variable_start, own_variable_start);
          layout_.components.push_back(runtime.block);
          runtime_components_.push_back(runtime);
        }
      }

      template <class ComponentT>
      LocalMapT makeLocalMap(const ComponentT& component,
                             IdxT              own_variable_start,
                             IdxT              own_equation_start)
      {
        LocalMapT map;
        map.own_variable_count = ComponentT::own_variable_count;
        map.own_equation_count = ComponentT::own_equation_count;

        for (std::size_t local = 0; local < ComponentT::own_variable_count; ++local)
        {
          map.variable_indices.push_back(own_variable_start + static_cast<IdxT>(local));
        }
        for (std::size_t local = 0; local < ComponentT::own_equation_count; ++local)
        {
          map.equation_indices.push_back(own_equation_start + static_cast<IdxT>(local));
          map.equation_accumulates.push_back(false);
        }

        for (std::size_t port = 0; port < ComponentT::port_count; ++port)
        {
          auto* bus = component.connectedBus(port);
          if (bus == nullptr)
          {
            throw std::runtime_error("EMT component port is not connected to a bus");
          }
          const auto& bus_block = busLayoutBlock(*bus);
          for (std::size_t phase = 0; phase < ComponentT::port_width(port); ++phase)
          {
            map.variable_indices.push_back(bus_block.variable_start + static_cast<IdxT>(phase));
            map.equation_indices.push_back(bus_block.equation_start + static_cast<IdxT>(phase));
            map.equation_accumulates.push_back(true);
          }
        }

        return map;
      }

      void buildSparseJacobianPattern()
      {
        std::vector<std::pair<IdxT, IdxT>>                       coordinates;
        std::vector<EMT::System::UnboundComponentJacobian<IdxT>> unbound_components;
        unbound_components.reserve(runtime_components_.size());
        derivative_columns_.clear();

        for (const auto& runtime : runtime_components_)
        {
          EMT::System::UnboundComponentJacobian<IdxT> unbound;
          switch (runtime.kind)
          {
          case EMT::System::ComponentKind::BranchLumpedConstant:
            unbound = EMT::System::SparseAD<IdxT>::analyze(*branches_.at(runtime.index),
                                                           runtime.block.local_map,
                                                           time_);
            break;
          case EMT::System::ComponentKind::LoadRL:
            unbound = EMT::System::SparseAD<IdxT>::analyze(*loads_.at(runtime.index),
                                                           runtime.block.local_map,
                                                           time_);
            break;
          case EMT::System::ComponentKind::VoltageSource:
            unbound = EMT::System::SparseAD<IdxT>::analyze(*sources_.at(runtime.index),
                                                           runtime.block.local_map,
                                                           time_);
            break;
          }

          coordinates.insert(coordinates.end(), unbound.coordinates.begin(), unbound.coordinates.end());
          derivative_columns_.insert(unbound.derivative_columns.begin(), unbound.derivative_columns.end());
          unbound_components.push_back(std::move(unbound));
        }

        csr_pattern_ = EMT::System::buildCsrPattern(size_, size_, coordinates);
        nnz_         = csr_pattern_.nnz();
        csr_rows_    = csr_pattern_.row_ptr;
        csr_cols_    = csr_pattern_.column_indices;
        csr_values_.assign(static_cast<std::size_t>(nnz_), RealT{0.0});

        for (std::size_t idx = 0; idx < runtime_components_.size(); ++idx)
        {
          runtime_components_[idx].jacobian =
              EMT::System::SparseAD<IdxT>::bind(unbound_components[idx], csr_pattern_);
        }

        csr_jac_ = std::make_unique<CsrMatrixT>(size_, size_, nnz_);
        csr_jac_->setDataPointers(csr_rows_.data(),
                                  csr_cols_.data(),
                                  csr_values_.data(),
                                  LinearAlgebra::memory::HOST);

        std::size_t max_local_variables = 0;
        std::size_t max_local_equations = 0;
        for (const auto& runtime : runtime_components_)
        {
          max_local_variables = std::max(max_local_variables,
                                         runtime.block.local_map.localVariableCount());
          max_local_equations = std::max(max_local_equations,
                                         runtime.block.local_map.localEquationCount());
        }
        local_y_.reserve(max_local_variables);
        local_yp_.reserve(max_local_variables);
        local_f_.reserve(max_local_equations);
        jac_local_y_.reserve(max_local_variables);
        jac_local_yp_.reserve(max_local_variables);
        jac_work_f_.reserve(max_local_equations);
      }

      template <class ComponentT>
      void copyOwnStateToGlobal(ComponentT& component)
      {
        for (std::size_t local = 0; local < ComponentT::own_variable_count; ++local)
        {
          const auto global = static_cast<std::size_t>(
              component.getVariableIndex(static_cast<IdxT>(local)));
          y_.at(global)  = component.y().at(local);
          yp_.at(global) = component.yp().at(local);
        }
      }

      const EMT::System::BusBlock<RealT, IdxT>& busLayoutBlock(const bus_type& bus) const
      {
        for (const auto& block : layout_.buses)
        {
          if (block.external_id == bus.busID())
          {
            return block;
          }
        }
        throw std::runtime_error("EMT bus is missing from layout");
      }

      void addMonitorBinding(std::unique_ptr<Model::VariableMonitorBase> binding)
      {
        if (binding != nullptr && !binding->empty())
        {
          monitor_.addMonitor(binding.get());
          monitor_bindings_.push_back(std::move(binding));
        }
      }

      void bindBusMonitor(const bus_type& bus)
      {
        using MonitorT     = Model::VariableMonitor<bus_type, BusData>;
        const auto label   = bus.name().empty() ? std::string{"bus"} : bus.name();
        auto       monitor = std::make_unique<MonitorT>(label, bus.data().monitored_variables);

        monitor->set(BusMonitorableVariables::va, [this, idx = globalBusIndex(bus, 0)]()
                     { return y_.at(idx); });
        monitor->set(BusMonitorableVariables::vb, [this, idx = globalBusIndex(bus, 1)]()
                     { return y_.at(idx); });
        monitor->set(BusMonitorableVariables::vc, [this, idx = globalBusIndex(bus, 2)]()
                     { return y_.at(idx); });
        monitor->set(BusMonitorableVariables::dva, [this, idx = globalBusIndex(bus, 0)]()
                     { return yp_.at(idx); });
        monitor->set(BusMonitorableVariables::dvb, [this, idx = globalBusIndex(bus, 1)]()
                     { return yp_.at(idx); });
        monitor->set(BusMonitorableVariables::dvc, [this, idx = globalBusIndex(bus, 2)]()
                     { return yp_.at(idx); });
        monitor->set(BusMonitorableVariables::ifa, [this, idx = globalBusIndex(bus, 0)]()
                     { return f_.at(idx); });
        monitor->set(BusMonitorableVariables::ifb, [this, idx = globalBusIndex(bus, 1)]()
                     { return f_.at(idx); });
        monitor->set(BusMonitorableVariables::ifc, [this, idx = globalBusIndex(bus, 2)]()
                     { return f_.at(idx); });
        addMonitorBinding(std::move(monitor));
      }

      std::size_t globalBusIndex(const bus_type& bus, IdxT phase) const
      {
        const auto& block = busLayoutBlock(bus);
        return static_cast<std::size_t>(block.variable_start + phase);
      }

      template <class ComponentT, template <typename, typename> typename DataT>
      std::unique_ptr<Model::VariableMonitor<ComponentT, DataT>>
      makeComponentMonitor(const typename ComponentT::DataT& data) const
      {
        return std::make_unique<Model::VariableMonitor<ComponentT, DataT>>(data);
      }

      void bindLoadMonitor(const load_type& load)
      {
        auto monitor = makeComponentMonitor<load_type, LoadRLData>(load.data());
        monitor->set(LoadRLMonitorableVariables::ia, [this, idx = globalOwnIndex(load, 0)]()
                     { return y_.at(idx); });
        monitor->set(LoadRLMonitorableVariables::ib, [this, idx = globalOwnIndex(load, 1)]()
                     { return y_.at(idx); });
        monitor->set(LoadRLMonitorableVariables::ic, [this, idx = globalOwnIndex(load, 2)]()
                     { return y_.at(idx); });
        monitor->set(LoadRLMonitorableVariables::dia, [this, idx = globalOwnIndex(load, 0)]()
                     { return yp_.at(idx); });
        monitor->set(LoadRLMonitorableVariables::dib, [this, idx = globalOwnIndex(load, 1)]()
                     { return yp_.at(idx); });
        monitor->set(LoadRLMonitorableVariables::dic, [this, idx = globalOwnIndex(load, 2)]()
                     { return yp_.at(idx); });
        addMonitorBinding(std::move(monitor));
      }

      void bindBranchMonitor(const branch_type& branch)
      {
        auto monitor = makeComponentMonitor<branch_type, BranchLumpedConstantData>(branch.data());
        monitor->set(BranchLumpedConstantMonitorableVariables::ia, [this, idx = globalOwnIndex(branch, 0)]()
                     { return y_.at(idx); });
        monitor->set(BranchLumpedConstantMonitorableVariables::ib, [this, idx = globalOwnIndex(branch, 1)]()
                     { return y_.at(idx); });
        monitor->set(BranchLumpedConstantMonitorableVariables::ic, [this, idx = globalOwnIndex(branch, 2)]()
                     { return y_.at(idx); });
        monitor->set(BranchLumpedConstantMonitorableVariables::dia, [this, idx = globalOwnIndex(branch, 0)]()
                     { return yp_.at(idx); });
        monitor->set(BranchLumpedConstantMonitorableVariables::dib, [this, idx = globalOwnIndex(branch, 1)]()
                     { return yp_.at(idx); });
        monitor->set(BranchLumpedConstantMonitorableVariables::dic, [this, idx = globalOwnIndex(branch, 2)]()
                     { return yp_.at(idx); });
        addMonitorBinding(std::move(monitor));
      }

      void bindVoltageSourceMonitor(const source_type& source)
      {
        auto* bus = source.connectedBus(0);
        if (bus == nullptr)
        {
          return;
        }

        auto monitor = makeComponentMonitor<source_type, VoltageSourceData>(source.data());
        monitor->set(VoltageSourceMonitorableVariables::ia, [this, &source, idx = globalBusIndex(*bus, 0)]()
                     { return sourceInjection(source, idx, 0); });
        monitor->set(VoltageSourceMonitorableVariables::ib, [this, &source, idx = globalBusIndex(*bus, 1)]()
                     { return sourceInjection(source, idx, 1); });
        monitor->set(VoltageSourceMonitorableVariables::ic, [this, &source, idx = globalBusIndex(*bus, 2)]()
                     { return sourceInjection(source, idx, 2); });
        addMonitorBinding(std::move(monitor));
      }

      template <class ComponentT>
      std::size_t globalOwnIndex(const ComponentT& component, IdxT local) const
      {
        return static_cast<std::size_t>(component.getVariableIndices().at(static_cast<std::size_t>(local)));
      }

      RealT sourceInjection(const source_type& source, std::size_t voltage_index, std::size_t phase) const
      {
        const RealT sqrt2 = std::sqrt(RealT{2.0});
        const RealT e_inst =
            sqrt2 * source.e()[phase] * std::cos(source.omega0() * time_ + source.phi()[phase]);
        return (e_inst - static_cast<RealT>(y_.at(voltage_index))) / source.r()[phase];
      }

      std::vector<bus_type*>       buses_;
      std::vector<component_type*> all_components_;
      std::vector<branch_type*>    branches_;
      std::vector<load_type*>      loads_;
      std::vector<source_type*>    sources_;

      std::map<std::string, IdxT> bus_name_to_index_;
      std::map<IdxT, IdxT>        gridkit_bus_indices_;

      bool owns_components_{false};

      EMT::System::Layout<RealT, IdxT> layout_;
      std::vector<RuntimeComponent>    runtime_components_;
      EMT::System::CsrPattern<IdxT>    csr_pattern_;
      std::set<IdxT>                   derivative_columns_;
      std::vector<IdxT>                csr_rows_;
      std::vector<IdxT>                csr_cols_;
      std::vector<RealT>               csr_values_;
      std::unique_ptr<CsrMatrixT>      csr_jac_;

      std::vector<ScalarT> local_y_;
      std::vector<ScalarT> local_yp_;
      std::vector<ScalarT> local_f_;
      std::vector<RealT>   jac_local_y_;
      std::vector<RealT>   jac_local_yp_;
      std::vector<RealT>   jac_work_f_;

      std::vector<std::unique_ptr<Model::VariableMonitorBase>> monitor_bindings_;
      Model::VariableMonitorController<ScalarT>                monitor_;
    };
  } // namespace EMT
} // namespace GridKit
