#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <GridKit/Model/EMT/ABCUtils.hpp>
#include <GridKit/Model/EMT/BusVoltageContribution.hpp>
#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/VariableMonitorController.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit::EMT
{
  using Log = ::GridKit::Utilities::Logger;

  template <typename scalar_type, typename index_type>
  SystemModel<scalar_type, index_type>::SystemModel()
    : monitor_(std::make_unique<MonitorT>(time_))
  {
    time_  = RealT{0.0};
    alpha_ = RealT{0.0};
  }

  template <typename scalar_type, typename index_type>
  SystemModel<scalar_type, index_type>::SystemModel(
      const SystemModelData<RealT, IdxT>& data)
    : monitor_(std::make_unique<MonitorT>(time_))
  {
    time_            = RealT{0.0};
    alpha_           = RealT{0.0};
    owns_components_ = true;

    for (const auto& bus_data : data.bus)
    {
      addBus(new BusT(bus_data));
    }

    for (const auto& signal_data : data.signal)
    {
      addSignal(new SignalT(signal_data));
    }

    using namespace PhasorDynamics;

    // Zero-state signal owners precede all consuming components.
    for (const auto& source_data : data.constant_source)
    {
      auto* source = new ConstantSignalSource<ScalarT, IdxT>(source_data);
      if (source_data.signal_outputs.contains(
              ConstantSignalSourceSignalOutputs::sr))
      {
        const auto signal_id = source_data.signal_outputs.at(
            ConstantSignalSourceSignalOutputs::sr);
        source->getSignals()
            .template assignSignalNode<
                ConstantSignalSourceInternalVariables::SREAL>(
                getSignal(signal_id));
      }
      if (source_data.signal_outputs.contains(
              ConstantSignalSourceSignalOutputs::si))
      {
        const auto signal_id = source_data.signal_outputs.at(
            ConstantSignalSourceSignalOutputs::si);
        source->getSignals()
            .template assignSignalNode<
                ConstantSignalSourceInternalVariables::SIMAG>(
                getSignal(signal_id));
      }
      addComponent(source, source_data.disambiguation_string);
    }

    for (const auto& source_data : data.voltage_source)
    {
      const auto bus_id = source_data.buses.at(VoltageSourceBuses::bus);
      addComponent(new VoltageSource<ScalarT, IdxT>(getBus(bus_id),
                                                    source_data),
                   source_data.disambiguation_string);
    }

    for (const auto& line_data : data.line_lumped)
    {
      const auto bus1_id = line_data.buses.at(LineLumpedBuses::bus1);
      const auto bus2_id = line_data.buses.at(LineLumpedBuses::bus2);
      addComponent(new LineLumped<ScalarT, IdxT>(getBus(bus1_id),
                                                 getBus(bus2_id),
                                                 line_data),
                   line_data.disambiguation_string);
    }

    for (const auto& load_data : data.loadz)
    {
      const auto bus_id    = load_data.buses.at(LoadZBuses::bus);
      auto*      load      = new LoadZ<ScalarT, IdxT>(getBus(bus_id),
                                            load_data);
      const auto signal_id = load_data.signal_inputs.at(
          LoadZSignalInputs::enable);
      load->getSignals()
          .template attachSignalNode<LoadZExternalVariables::enable>(
              getSignal(signal_id));
      addComponent(load, load_data.disambiguation_string);
    }

    for (const auto& vector_fit_data : data.vector_fit)
    {
      auto* vector_fit = new VectorFit<ScalarT, IdxT>(vector_fit_data);
      vector_fit->getSignals()
          .template attachSignalNode<VectorFitExternalVariables::input_a>(
              getSignal(vector_fit_data.signal_inputs.at(
                  VectorFitSignalInputs::input_a)));
      vector_fit->getSignals()
          .template attachSignalNode<VectorFitExternalVariables::input_b>(
              getSignal(vector_fit_data.signal_inputs.at(
                  VectorFitSignalInputs::input_b)));
      vector_fit->getSignals()
          .template attachSignalNode<VectorFitExternalVariables::input_c>(
              getSignal(vector_fit_data.signal_inputs.at(
                  VectorFitSignalInputs::input_c)));
      vector_fit->getSignals()
          .template assignSignalNode<VectorFitInternalVariables::out_a>(
              getSignal(vector_fit_data.signal_outputs.at(
                  VectorFitSignalOutputs::out_a)));
      vector_fit->getSignals()
          .template assignSignalNode<VectorFitInternalVariables::out_b>(
              getSignal(vector_fit_data.signal_outputs.at(
                  VectorFitSignalOutputs::out_b)));
      vector_fit->getSignals()
          .template assignSignalNode<VectorFitInternalVariables::out_c>(
              getSignal(vector_fit_data.signal_outputs.at(
                  VectorFitSignalOutputs::out_c)));
      addComponent(vector_fit, vector_fit_data.disambiguation_string);
    }

    for (const auto& sink : data.monitor_sink)
    {
      monitor_->addSink(sink);
    }
  }

  template <typename scalar_type, typename index_type>
  SystemModel<scalar_type, index_type>::~SystemModel()
  {
    if (!owns_components_)
    {
      return;
    }

    for (auto* component : components_)
    {
      delete component;
    }
    for (auto* bus : buses_)
    {
      delete bus;
    }
    for (auto* signal : signals_)
    {
      delete signal;
    }
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::setGridKitComponentID(
      IdxT component_id)
  {
    gridkit_component_id_ = component_id;
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::allocate()
  {
    size_ = 0;
    for (auto* bus : buses_)
    {
      size_ += bus->size();
    }
    for (auto* component : components_)
    {
      size_ += component->size();
    }

    if (!allocated_)
    {
      delete csr_jac_;
      csr_jac_ = nullptr;
      delete[] map_to_csr_;
      map_to_csr_ = nullptr;
      nnz_        = 0;
      this->allocateVectors(size_);
    }

    if (y_.getSize() != size_ || yp_.getSize() != size_
        || f_.getSize() != size_ || abs_tol_.getSize() != size_)
    {
      throw std::runtime_error("EMT SystemModel vector size mismatch");
    }

    tag_.resize(size_);
    variable_indices_.resize(size_);
    residual_indices_.resize(size_);
    for (IdxT j = 0; j < size_; ++j)
    {
      this->setVariableIndex(j, j);
      this->setResidualIndex(j, j);
    }

    IdxT offset = 0;
    for (auto* bus : buses_)
    {
      if (bus->bind(y_, yp_, f_, abs_tol_, offset) != 0
          || bus->allocate() != 0)
      {
        throw std::runtime_error("Failed to allocate EMT bus");
      }
      for (IdxT j = 0; j < bus->size(); ++j)
      {
        bus->setVariableIndex(j, offset + j);
        bus->setResidualIndex(j, offset + j);
      }
      offset += bus->size();
    }

    for (auto* component : components_)
    {
      if (component->bind(y_, yp_, f_, abs_tol_, offset) != 0
          || component->allocate() != 0)
      {
        throw std::runtime_error("Failed to allocate EMT component");
      }
      for (IdxT j = 0; j < component->size(); ++j)
      {
        component->setVariableIndex(j, offset + j);
        component->setResidualIndex(j, offset + j);
      }
      offset += component->size();
    }

    if (offset != size_ || classifyBusVoltages() != 0 || verify() != 0)
    {
      throw std::runtime_error("EMT SystemModel verification failed");
    }

    initializeMonitor();
    startMonitor();

    if (hasJacobian())
    {
      evaluateResidual();
      evaluateJacobian();
    }

    allocated_ = true;
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::verify() const
  {
    int errors = 0;
    for (const auto* bus : buses_)
    {
      errors += bus->verify();
    }
    for (const auto* component : components_)
    {
      errors += component->verify();
    }

    return errors;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::classifyBusVoltages()
  {
    std::map<IdxT, ABCMatrix<RealT>> derivative_gram;
    std::map<IdxT, ABCMatrix<RealT>> algebraic_gram;
    std::map<IdxT, std::size_t>      connection_count;
    for (const auto* bus : buses_)
    {
      derivative_gram.emplace(bus->busID(), ABCMatrix<RealT>{});
      algebraic_gram.emplace(bus->busID(), ABCMatrix<RealT>{});
      connection_count.emplace(bus->busID(), 0);
    }

    std::vector<BusVoltageContribution<ScalarT, IdxT>> contributions;
    for (const auto* component : components_)
    {
      const auto* contributor =
          dynamic_cast<const BusVoltageContributor<ScalarT, IdxT>*>(component);
      if (contributor != nullptr)
      {
        contributor->appendBusVoltageContributions(contributions);
      }
    }

    for (const auto& contribution : contributions)
    {
      const auto derivative_entry = derivative_gram.find(contribution.bus_id);
      const auto algebraic_entry  = algebraic_gram.find(contribution.bus_id);
      if (derivative_entry == derivative_gram.end()
          || algebraic_entry == algebraic_gram.end())
      {
        Log::error() << "EMT::SystemModel: voltage contribution references "
                        "unknown bus "
                     << contribution.bus_id << '\n';
        return 1;
      }

      if (!Detail::accumulateNormalizedRowGram(
              derivative_entry->second,
              contribution.derivative_block)
          || !Detail::accumulateNormalizedRowGram(
              algebraic_entry->second,
              contribution.algebraic_block))
      {
        Log::error() << "EMT::SystemModel: equation contribution for bus "
                     << contribution.bus_id << " must be finite\n";
        return 1;
      }
      ++connection_count.at(contribution.bus_id);
    }

    int errors = 0;
    for (auto* bus : buses_)
    {
      const auto rank = Detail::matrixRank(derivative_gram.at(bus->busID()));
      if (rank == 0)
      {
        const auto algebraic_rank = Detail::matrixRank(
            algebraic_gram.at(bus->busID()));
        if (algebraic_rank == 0
            && connection_count.at(bus->busID()) == 0)
        {
          Log::error() << "EMT::SystemModel: bus " << bus->busID()
                       << " has no connected current equation\n";
          ++errors;
        }
        else if (algebraic_rank == 0 || algebraic_rank == 3)
        {
          bus->setVoltageClass(BusVoltageClass::algebraic);
          bus->setKCLDifferentiationRequired(algebraic_rank == 0);
        }
        else
        {
          Log::error() << "EMT::SystemModel: bus " << bus->busID()
                       << " has unsupported algebraic bus-voltage rank "
                       << algebraic_rank << '\n';
          ++errors;
        }
      }
      else if (rank == 3)
      {
        bus->setVoltageClass(BusVoltageClass::differential);
        bus->setKCLDifferentiationRequired(false);
      }
      else
      {
        Log::error() << "EMT::SystemModel: bus " << bus->busID()
                     << " has unsupported bus-voltage derivative rank "
                     << rank << '\n';
        ++errors;
      }
    }
    return errors;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::initialize()
  {
    for (auto* bus : buses_)
    {
      bus->initialize();
    }
    for (auto* component : components_)
    {
      component->initialize();
    }
    y_.setDataUpdated();
    yp_.setDataUpdated();
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::validateInitialState()
  {
    for (auto* bus : buses_)
    {
      bus->setOriginalKCLValidation(true);
    }

    if (evaluateResidual() != 0)
    {
      for (auto* bus : buses_)
      {
        bus->setOriginalKCLValidation(false);
      }
      return 1;
    }

    int errors{0};
    for (auto* bus : buses_)
    {
      if (!bus->kcl_differentiation_required_)
      {
        continue;
      }

      const auto* kcl = bus->getResidual().getData();
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        const RealT value     = Detail::scalarValue<RealT>(kcl[phase]);
        const RealT tolerance = RealT{1024.0}
                                * std::numeric_limits<RealT>::epsilon()
                                * std::max(RealT{1.0},
                                           bus->current_scale_[phase]);
        if (!std::isfinite(value) || std::abs(value) > tolerance)
        {
          Log::error() << "EMT::SystemModel: initial state violates KCL at "
                          "algebraic bus "
                       << bus->busID() << '\n';
          ++errors;
          break;
        }
      }
    }

    for (auto* bus : buses_)
    {
      bus->setOriginalKCLValidation(false);
    }
    return errors;
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::initializeMonitor()
  {
    for (std::size_t index = monitored_bus_count_;
         index < buses_.size();
         ++index)
    {
      const auto* bus               = buses_[index];
      const auto* component_monitor = bus->getMonitor();
      if (component_monitor != nullptr && !component_monitor->empty())
      {
        monitor_->addMonitor(component_monitor);
      }
    }
    monitored_bus_count_ = buses_.size();

    for (std::size_t index = monitored_component_count_;
         index < components_.size();
         ++index)
    {
      const auto* component         = components_[index];
      const auto* component_monitor = component->getMonitor();
      if (component_monitor != nullptr && !component_monitor->empty())
      {
        monitor_->addMonitor(component_monitor);
      }
    }
    monitored_component_count_ = components_.size();
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::startMonitor()
  {
    if (monitor_started_ || monitor_->empty())
    {
      return;
    }
    monitor_->start();
    monitor_started_ = true;
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::stopMonitor()
  {
    if (!monitor_started_)
    {
      return;
    }
    monitor_->stop();
    monitor_started_ = false;
  }

  template <typename scalar_type, typename index_type>
  bool SystemModel<scalar_type, index_type>::monitoring() const
  {
    return !monitor_->empty();
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::printMonitoredVariables() const
  {
    monitor_->print();
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::tagDifferentiable()
  {
    for (auto* bus : buses_)
    {
      bus->tagDifferentiable();
      for (IdxT j = 0; j < bus->size(); ++j)
      {
        tag_[bus->getVariableIndex(j)] = bus->tag()[j];
      }
    }
    for (auto* component : components_)
    {
      component->tagDifferentiable();
      for (IdxT j = 0; j < component->size(); ++j)
      {
        tag_[component->getVariableIndex(j)] = component->tag()[j];
      }
    }
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::setAbsoluteTolerance(RealT rel_tol)
  {
    for (auto* bus : buses_)
    {
      bus->setAbsoluteTolerance(rel_tol);
    }
    for (auto* component : components_)
    {
      component->setAbsoluteTolerance(rel_tol);
    }
    abs_tol_.setDataUpdated();
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::evaluateResidual()
  {
    for (auto* bus : buses_)
    {
      bus->evaluateResidual();
    }
    for (auto* component : components_)
    {
      component->evaluateResidual();
    }
    f_.setDataUpdated();
    return 0;
  }

  template <typename scalar_type, typename index_type>
  int SystemModel<scalar_type, index_type>::evaluateJacobian()
  {
    for (auto* bus : buses_)
    {
      bus->evaluateJacobian();
    }
    for (auto* component : components_)
    {
      component->evaluateJacobian();
    }

    if (csr_jac_ == nullptr)
    {
      IdxT nnz_with_duplicates = 0;
      for (auto* component : components_)
      {
        auto* jacobian = component->getCooJacobian();
        if (jacobian != nullptr)
        {
          nnz_with_duplicates += jacobian->getNnz();
        }
      }
      for (auto* bus : buses_)
      {
        auto* jacobian = bus->getCooJacobian();
        if (jacobian != nullptr)
        {
          nnz_with_duplicates += jacobian->getNnz();
        }
      }

      auto* rows = new IdxT[static_cast<std::size_t>(nnz_with_duplicates)];
      auto* cols = new IdxT[static_cast<std::size_t>(nnz_with_duplicates)];
      auto* vals = new RealT[static_cast<std::size_t>(nnz_with_duplicates)];

      IdxT       counter = 0;
      const auto append  = [&](auto* owner)
      {
        auto* jacobian = owner->getCooJacobian();
        if (jacobian == nullptr)
        {
          return;
        }
        for (IdxT entry = 0; entry < jacobian->getNnz(); ++entry)
        {
          rows[counter] = jacobian->getRowData()[entry];
          cols[counter] = jacobian->getColData()[entry];
          vals[counter] = jacobian->getValues()[entry];
          ++counter;
        }
      };
      for (auto* component : components_)
      {
        append(component);
      }
      for (auto* bus : buses_)
      {
        append(bus);
      }

      CooMatrixT coo(size_, size_, nnz_with_duplicates, &rows, &cols, &vals);
      auto*      row_ptrs = coo.getCsrRowData();
      nnz_                = coo.getNnz();

      auto* csr_cols = new IdxT[static_cast<std::size_t>(nnz_)];
      auto* csr_vals = new RealT[static_cast<std::size_t>(nnz_)];
      std::copy(coo.getColData(), coo.getColData() + nnz_, csr_cols);
      std::copy(coo.getValues(), coo.getValues() + nnz_, csr_vals);
      csr_jac_ = new CsrMatrixT(size_, size_, nnz_, &row_ptrs, &csr_cols, &csr_vals);

      const auto* map_to_sorted = coo.getMapToSorted();
      const auto* map_to_dedup  = coo.getMapToDeduplicated();
      map_to_csr_ =
          new IdxT[static_cast<std::size_t>(nnz_with_duplicates)];
      for (IdxT entry = 0; entry < nnz_with_duplicates; ++entry)
      {
        map_to_csr_[map_to_sorted[entry]] = map_to_dedup[entry];
      }
      jacobian_entry_count_ = nnz_with_duplicates;
    }
    else
    {
      auto* values = csr_jac_->getValues();
      std::fill(values, values + csr_jac_->getNnz(), RealT{0.0});

      // The assembly map is built once, so every owner must emit the same
      // number of entries in the same order on every later evaluation.
      IdxT       counter    = 0;
      const auto accumulate = [&](auto* owner)
      {
        auto* jacobian = owner->getCooJacobian();
        if (jacobian == nullptr)
        {
          return;
        }
        if (jacobian->getNnz() > jacobian_entry_count_ - counter)
        {
          throw std::runtime_error(
              "EMT SystemModel Jacobian sparsity pattern changed after assembly");
        }
        for (IdxT entry = 0; entry < jacobian->getNnz(); ++entry)
        {
          values[map_to_csr_[counter]] += jacobian->getValues()[entry];
          ++counter;
        }
      };
      for (auto* component : components_)
      {
        accumulate(component);
      }
      for (auto* bus : buses_)
      {
        accumulate(bus);
      }
      if (counter != jacobian_entry_count_)
      {
        throw std::runtime_error(
            "EMT SystemModel Jacobian sparsity pattern changed after assembly");
      }
    }
    return 0;
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::updateTime(RealT time,
                                                        RealT alpha)
  {
    time_  = time;
    alpha_ = alpha;
    for (auto* component : components_)
    {
      component->updateTime(time, alpha);
    }
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::addBus(BusT* bus)
  {
    if (bus == nullptr || gridkit_bus_indices_.contains(bus->busID()))
    {
      throw std::invalid_argument("Null or duplicate EMT bus");
    }
    const auto local_id = static_cast<IdxT>(buses_.size());
    stopMonitor();
    gridkit_bus_indices_[bus->busID()] = local_id;
    buses_.push_back(bus);
    allocated_ = false;
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::addSignal(SignalT* signal)
  {
    if (signal == nullptr
        || gridkit_signal_indices_.contains(signal->signalId()))
    {
      throw std::invalid_argument("Null or duplicate EMT signal");
    }
    const auto local_id                         = static_cast<IdxT>(signals_.size());
    gridkit_signal_indices_[signal->signalId()] = local_id;
    signals_.push_back(signal);
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::addComponent(
      ComponentT* component)
  {
    if (component == nullptr)
    {
      throw std::invalid_argument("Null EMT component");
    }
    stopMonitor();
    component->setGridKitComponentID(
        static_cast<IdxT>(components_.size()));
    component->updateTime(time_, alpha_);
    components_.push_back(component);
    allocated_ = false;
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::addComponent(
      ComponentT*        component,
      const std::string& component_id)
  {
    if (component_id.empty()
        || gridkit_component_indices_.contains(component_id))
    {
      throw std::invalid_argument("Empty or duplicate EMT component ID");
    }

    addComponent(component);
    gridkit_component_indices_[component_id] =
        component->getGridKitComponentID();
  }

  template <typename scalar_type, typename index_type>
  typename SystemModel<scalar_type, index_type>::BusT*
  SystemModel<scalar_type, index_type>::getBus(IdxT bus_id)
  {
    const auto local_id = gridkit_bus_indices_.at(bus_id);
    assert(buses_[local_id]->busID() == bus_id);
    return buses_[local_id];
  }

  template <typename scalar_type, typename index_type>
  typename SystemModel<scalar_type, index_type>::SignalT*
  SystemModel<scalar_type, index_type>::getSignal(IdxT signal_id)
  {
    const auto local_id = gridkit_signal_indices_.at(signal_id);
    assert(signals_[local_id]->signalId() == signal_id);
    return signals_[local_id];
  }

  template <typename scalar_type, typename index_type>
  typename SystemModel<scalar_type, index_type>::ComponentT*
  SystemModel<scalar_type, index_type>::getComponent(
      IdxT gridkit_component_id)
  {
    return components_.at(gridkit_component_id);
  }

  template <typename scalar_type, typename index_type>
  typename SystemModel<scalar_type, index_type>::ComponentT*
  SystemModel<scalar_type, index_type>::getComponent(
      const std::string& component_id)
  {
    return getComponent(gridkit_component_indices_.at(component_id));
  }
} // namespace GridKit::EMT
