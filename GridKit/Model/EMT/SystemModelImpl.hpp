#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>

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
    : omega0_(data.omega0), monitor_(std::make_unique<MonitorT>(time_))
  {
    time_            = RealT{0.0};
    alpha_           = RealT{0.0};
    owns_components_ = true;

    for (const auto& bus_data : data.bus)
    {
      addBus(new BusT(bus_data, omega0_));
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
      addComponent(source);
    }

    for (const auto& source_data : data.voltage_source)
    {
      const auto bus_id = source_data.buses.at(VoltageSourceBuses::bus);
      addComponent(new VoltageSource<ScalarT, IdxT>(getBus(bus_id),
                                                    source_data));
    }

    for (const auto& line_data : data.line_lumped)
    {
      const auto bus1_id = line_data.buses.at(LineLumpedBuses::bus1);
      const auto bus2_id = line_data.buses.at(LineLumpedBuses::bus2);
      addComponent(new LineLumped<ScalarT, IdxT>(getBus(bus1_id),
                                                 getBus(bus2_id),
                                                 line_data,
                                                 omega0_));
    }

    for (const auto& load_data : data.loadz)
    {
      const auto bus_id    = load_data.buses.at(LoadZBuses::bus);
      auto*      load      = new LoadZ<ScalarT, IdxT>(getBus(bus_id),
                                            load_data,
                                            omega0_);
      const auto signal_id = load_data.signal_inputs.at(
          LoadZSignalInputs::enable);
      load->getSignals()
          .template attachSignalNode<LoadZExternalVariables::enable>(
              getSignal(signal_id));
      addComponent(load);
    }

    for (const auto& vector_fit_data : data.vector_fit)
    {
      auto* vector_fit = new VectorFit<ScalarT, IdxT>(vector_fit_data,
                                                      omega0_);
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
      addComponent(vector_fit);
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

    if (offset != size_ || verify() != 0)
    {
      throw std::runtime_error("EMT SystemModel verification failed");
    }

    initializeMonitor();
    startMonitor();

    if (hasJacobian())
    {
      initialize();
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
  void SystemModel<scalar_type, index_type>::initializeMonitor()
  {
    for (const auto* bus : buses_)
    {
      const auto* component_monitor = bus->getMonitor();
      if (component_monitor != nullptr && !component_monitor->empty())
      {
        monitor_->addMonitor(component_monitor);
      }
    }
    for (const auto* component : components_)
    {
      const auto* component_monitor = component->getMonitor();
      if (component_monitor != nullptr && !component_monitor->empty())
      {
        monitor_->addMonitor(component_monitor);
      }
    }
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::startMonitor()
  {
    monitor_->start();
  }

  template <typename scalar_type, typename index_type>
  void SystemModel<scalar_type, index_type>::stopMonitor()
  {
    monitor_->stop();
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
    }
    else
    {
      auto* values = csr_jac_->getValues();
      std::fill(values, values + csr_jac_->getNnz(), RealT{0.0});

      IdxT       counter    = 0;
      const auto accumulate = [&](auto* owner)
      {
        auto* jacobian = owner->getCooJacobian();
        if (jacobian == nullptr)
        {
          return;
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
    const auto local_id                = static_cast<IdxT>(buses_.size());
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
    component->setGridKitComponentID(
        static_cast<IdxT>(components_.size()));
    component->updateTime(time_, alpha_);
    components_.push_back(component);
    allocated_ = false;
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
} // namespace GridKit::EMT
