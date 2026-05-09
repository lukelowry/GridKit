/**
 * @file SystemModelImpl.hpp
 * @brief Implementation of the EMT system model.
 */

#pragma once

#include <algorithm>
#include <map>
#include <stdexcept>

#include <GridKit/LinearAlgebra/SparseMatrix/CooMatrix.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    SystemModel<ScalarT, IdxT>::SystemModel(const SystemModelDataT& data)
    {
      std::map<IdxT, BusT*> buses;

      for (const auto& bus_data : data.bus)
      {
        auto& bus = addBus(bus_data.data);
        if (!buses.emplace(bus_data.number, &bus).second)
        {
          throw std::runtime_error("EMT::SystemModel: duplicate bus number "
                                   + std::to_string(bus_data.number));
        }
      }

      auto busByNumber = [&](IdxT number) -> BusT&
      {
        auto it = buses.find(number);
        if (it == buses.end())
        {
          throw std::runtime_error("EMT::SystemModel: unknown bus number "
                                   + std::to_string(number));
        }
        return *it->second;
      };

      for (const auto& line_data : data.line)
      {
        auto& line       = addLine(line_data.data);
        line_ids_.back() = line_data.id;
        const auto ret   = connect(line.terminal(0), busByNumber(line_data.bus1))
                         + connect(line.terminal(1), busByNumber(line_data.bus2));
        if (ret != 0)
        {
          throw std::runtime_error("EMT::SystemModel: failed to connect line '"
                                   + line_data.id + "'");
        }
      }

      for (const auto& load_data : data.shunt_load)
      {
        auto& load             = addShuntLoad(load_data.data);
        shunt_load_ids_.back() = load_data.id;
        if (connect(load.terminal(0), busByNumber(load_data.bus)) != 0)
        {
          throw std::runtime_error("EMT::SystemModel: failed to connect shunt load '"
                                   + load_data.id + "'");
        }
      }

      for (const auto& source_data : data.voltage_source)
      {
        auto& source               = addVoltageSource(source_data.data);
        voltage_source_ids_.back() = source_data.id;
        if (connect(source.terminal(0), busByNumber(source_data.bus)) != 0)
        {
          throw std::runtime_error("EMT::SystemModel: failed to connect voltage source '"
                                   + source_data.id + "'");
        }
      }
    }

    template <class ScalarT, typename IdxT>
    Bus<ScalarT, IdxT>& SystemModel<ScalarT, IdxT>::addBus(const BusDataT& data)
    {
      ensureMutableTopology();

      auto  bus = std::make_unique<BusT>(data);
      auto* ptr = bus.get();
      buses_.push_back(std::move(bus));
      return *ptr;
    }

    template <class ScalarT, typename IdxT>
    Line<ScalarT, IdxT>& SystemModel<ScalarT, IdxT>::addLine(const LineDataT& data)
    {
      ensureMutableTopology();

      auto  line = std::make_unique<LineT>(data);
      auto* ptr  = line.get();
      components_.push_back(std::move(line));
      lines_.push_back(ptr);
      line_ids_.emplace_back();
      return *ptr;
    }

    template <class ScalarT, typename IdxT>
    ShuntLoad<ScalarT, IdxT>& SystemModel<ScalarT, IdxT>::addShuntLoad(const ShuntLoadDataT& data)
    {
      ensureMutableTopology();

      auto  load = std::make_unique<ShuntLoadT>(data);
      auto* ptr  = load.get();
      components_.push_back(std::move(load));
      shunt_loads_.push_back(ptr);
      shunt_load_ids_.emplace_back();
      return *ptr;
    }

    template <class ScalarT, typename IdxT>
    VoltageSource<ScalarT, IdxT>& SystemModel<ScalarT, IdxT>::addVoltageSource(
        const VoltageSourceDataT& data)
    {
      ensureMutableTopology();

      auto  source = std::make_unique<VoltageSourceT>(data);
      auto* ptr    = source.get();
      components_.push_back(std::move(source));
      voltage_sources_.push_back(ptr);
      voltage_source_ids_.emplace_back();
      return *ptr;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::connect(TerminalRef terminal, BusT& bus)
    {
      ensureMutableTopology();

      if (terminal.component == nullptr || terminal.index >= terminal.component->terminalCount())
      {
        return 1;
      }

      connections_.push_back({terminal, &bus});
      return 0;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getLine(const std::string& id) -> LineT*
    {
      for (size_t i = 0; i < line_ids_.size(); ++i)
      {
        if (!id.empty() && line_ids_[i] == id)
        {
          return lines_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getLine(const std::string& id) const -> const LineT*
    {
      for (size_t i = 0; i < line_ids_.size(); ++i)
      {
        if (!id.empty() && line_ids_[i] == id)
        {
          return lines_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getShuntLoad(const std::string& id) -> ShuntLoadT*
    {
      for (size_t i = 0; i < shunt_load_ids_.size(); ++i)
      {
        if (!id.empty() && shunt_load_ids_[i] == id)
        {
          return shunt_loads_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getShuntLoad(const std::string& id) const
        -> const ShuntLoadT*
    {
      for (size_t i = 0; i < shunt_load_ids_.size(); ++i)
      {
        if (!id.empty() && shunt_load_ids_[i] == id)
        {
          return shunt_loads_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getVoltageSource(const std::string& id) -> VoltageSourceT*
    {
      for (size_t i = 0; i < voltage_source_ids_.size(); ++i)
      {
        if (!id.empty() && voltage_source_ids_[i] == id)
        {
          return voltage_sources_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getVoltageSource(const std::string& id) const
        -> const VoltageSourceT*
    {
      for (size_t i = 0; i < voltage_source_ids_.size(); ++i)
      {
        if (!id.empty() && voltage_source_ids_[i] == id)
        {
          return voltage_sources_[i];
        }
      }
      return nullptr;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::cue(const std::string& target, Action action)
    {
      if (auto* load = getShuntLoad(target); load != nullptr)
      {
        load->apply(action);
        return;
      }

      throw std::runtime_error("EMT::SystemModel: no cue target with id '" + target + "'");
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::allocate()
    {
      if (allocated_)
      {
        return 0;
      }

      IdxT offset = 0;
      for (auto& bus : buses_)
      {
        bus->allocate();
        const auto bus_size = bus->size();
        bus->setIndexRanges({offset, bus_size}, {offset, bus_size});
        offset += bus_size;
      }

      for (auto& component : components_)
      {
        component->allocate();
        const auto component_size = component->size();
        component->setIndexRanges({offset, component_size}, {offset, component_size});
        offset += component_size;
      }

      size_ = offset;
      allocateVectors();
      J_ = MatrixT(size_, size_);
      J_.reserve(size_ > 0 ? size_ * size_ : 0);

      const auto errors = verify();
      if (errors > 0)
      {
        Log::error() << "EMT::SystemModel: component verification failed with "
                     << errors << " errors\n";
        return errors;
      }

      const auto bind_errors = bindConnections();
      if (bind_errors > 0)
      {
        Log::error() << "EMT::SystemModel: connection binding failed with "
                     << bind_errors << " errors\n";
        return bind_errors;
      }

      const auto jacobian_errors = evaluateJacobian();
      if (jacobian_errors > 0)
      {
        Log::error() << "EMT::SystemModel: initial Jacobian assembly failed with "
                     << jacobian_errors << " errors\n";
        return jacobian_errors;
      }

      allocated_ = true;
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::verify() const
    {
      int ret = 0;
      for (const auto& bus : buses_)
      {
        ret += bus->verify();
      }
      for (const auto& component : components_)
      {
        ret += component->verify();
      }
      ret += verifyConnections();
      return ret;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::initialize()
    {
      int ret = 0;

      for (auto& bus : buses_)
      {
        ret              += bus->initialize();
        const auto range  = bus->variableRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          y_[range.begin + i]  = bus->y()[static_cast<size_t>(i)];
          yp_[range.begin + i] = bus->yp()[static_cast<size_t>(i)];
        }
      }

      for (auto& component : components_)
      {
        ret              += component->initialize();
        const auto range  = component->variableRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          y_[range.begin + i]  = component->y()[static_cast<size_t>(i)];
          yp_[range.begin + i] = component->yp()[static_cast<size_t>(i)];
        }
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::tagDifferentiable()
    {
      int ret = 0;
      std::fill(tag_.begin(), tag_.end(), false);

      for (auto& bus : buses_)
      {
        ret += bus->tagDifferentiable();
        bus->stampDifferentiable(tag_);
      }
      for (auto& component : components_)
      {
        ret += component->tagDifferentiable();
        component->stampDifferentiable(tag_);
        stampTerminalDifferentiability(*component);
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::evaluateResidual()
    {
      int ret = 0;

      distributeVectors();
      std::fill(f_.begin(), f_.end(), static_cast<ScalarT>(0.0));

      for (auto& bus : buses_)
      {
        ret += bus->evaluateResidual();
        bus->stampResidual(f_);
      }
      for (auto& component : components_)
      {
        ret += component->evaluateResidual();
        component->stampResidual(f_);
        assembleTerminalCurrents(*component);
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::evaluateJacobian()
    {
      int ret = 0;

      distributeVectors();

      J_.zeroMatrix();
      for (auto& bus : buses_)
      {
        ret += bus->evaluateJacobian();
        appendJacobianEntries(*bus);
      }
      for (auto& component : components_)
      {
        ret += component->evaluateJacobian();
        appendJacobianEntries(*component);
      }

      if (csr_jac_ == nullptr)
      {
        assembleCsrFromCurrentJacobians();
      }
      else
      {
        updateCsrValuesFromCurrentJacobians();
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    bool SystemModel<ScalarT, IdxT>::hasJacobian()
    {
      return true;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::updateTime(RealT t, RealT a)
    {
      time_  = t;
      alpha_ = a;
      for (auto& bus : buses_)
      {
        bus->updateTime(t, a);
      }
      for (auto& component : components_)
      {
        component->updateTime(t, a);
      }
    }

    template <class ScalarT, typename IdxT>
    IdxT SystemModel<ScalarT, IdxT>::size()
    {
      return size_;
    }

    template <class ScalarT, typename IdxT>
    IdxT SystemModel<ScalarT, IdxT>::nnz()
    {
      return nnz_;
    }

    template <class ScalarT, typename IdxT>
    IdxT SystemModel<ScalarT, IdxT>::sizeQuadrature()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    IdxT SystemModel<ScalarT, IdxT>::sizeParams()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::setTolerances(RealT& rtol, RealT& atol) const
    {
      rtol = rel_tol_;
      atol = abs_tol_;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::setMaxSteps(IdxT& msa) const
    {
      msa = max_steps_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::y()
    {
      return y_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::y() const
    {
      return y_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::yp()
    {
      return yp_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::yp() const
    {
      return yp_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<bool>& SystemModel<ScalarT, IdxT>::tag()
    {
      return tag_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<bool>& SystemModel<ScalarT, IdxT>::tag() const
    {
      return tag_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::yB()
    {
      return yB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::yB() const
    {
      return yB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::ypB()
    {
      return ypB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::ypB() const
    {
      return ypB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param()
    {
      return param_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param() const
    {
      return param_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param_up()
    {
      return param_up_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param_up() const
    {
      return param_up_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param_lo()
    {
      return param_lo_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::param_lo() const
    {
      return param_lo_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getResidual()
    {
      return f_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getResidual() const
    {
      return f_;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getJacobian() -> MatrixT&
    {
      return J_;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getJacobian() const -> const MatrixT&
    {
      return J_;
    }

    template <class ScalarT, typename IdxT>
    auto SystemModel<ScalarT, IdxT>::getCsrJacobian() const -> CsrMatrixT*
    {
      return csr_jac_.get();
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::evaluateIntegrand()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getIntegrand()
    {
      return g_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getIntegrand() const
    {
      return g_;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::initializeAdjoint()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::evaluateAdjointResidual()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::evaluateAdjointIntegrand()
    {
      return 0;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getAdjointResidual()
    {
      return fB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getAdjointResidual() const
    {
      return fB_;
    }

    template <class ScalarT, typename IdxT>
    std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getAdjointIntegrand()
    {
      return gB_;
    }

    template <class ScalarT, typename IdxT>
    const std::vector<ScalarT>& SystemModel<ScalarT, IdxT>::getAdjointIntegrand() const
    {
      return gB_;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::ensureMutableTopology() const
    {
      if (allocated_)
      {
        throw std::logic_error("EMT::SystemModel topology is frozen after allocate()");
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::allocateVectors()
    {
      const auto size = static_cast<size_t>(size_);
      y_.resize(size);
      yp_.resize(size);
      f_.resize(size);
      tag_.resize(size);
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::verifyConnections() const
    {
      int ret = 0;

      for (const auto& connection : connections_)
      {
        if (connection.terminal.component == nullptr || connection.bus == nullptr)
        {
          Log::error() << "EMT::SystemModel: connection endpoints must not be null\n";
          ret += 1;
          continue;
        }
        if (connection.terminal.index >= connection.terminal.component->terminalCount())
        {
          Log::error() << "EMT::SystemModel: connection terminal index is out of range\n";
          ret += 1;
        }
        const auto terminal_is_owned =
            std::any_of(components_.begin(),
                        components_.end(),
                        [&](const auto& component)
                        {
                          return component.get() == connection.terminal.component;
                        });
        if (!terminal_is_owned)
        {
          Log::error()
              << "EMT::SystemModel: connection terminal component is not owned by this system\n";
          ret += 1;
        }
        const auto bus_is_owned =
            std::any_of(buses_.begin(),
                        buses_.end(),
                        [&](const auto& bus)
                        {
                          return bus.get() == connection.bus;
                        });
        if (!bus_is_owned)
        {
          Log::error() << "EMT::SystemModel: connection bus is not owned by this system\n";
          ret += 1;
        }
      }

      for (const auto& component : components_)
      {
        for (size_t terminal = 0; terminal < component->terminalCount(); ++terminal)
        {
          size_t count = 0;
          for (const auto& connection : connections_)
          {
            if (connection.terminal.component == component.get()
                && connection.terminal.index == terminal)
            {
              ++count;
            }
          }
          if (count != 1)
          {
            Log::error()
                << "EMT::SystemModel: each component terminal must have exactly "
                << "one bus connection\n";
            ret += 1;
          }
        }
      }

      return ret;
    }

    template <class ScalarT, typename IdxT>
    int SystemModel<ScalarT, IdxT>::bindConnections()
    {
      int ret = 0;
      for (auto& connection : connections_)
      {
        typename ComponentT::TerminalView view;
        view.y               = connection.bus->y().data();
        view.yp              = connection.bus->yp().data();
        view.variable_index  = connection.bus->variableIndex(0);
        view.residual_index  = connection.bus->residualIndex(0);
        ret                 += connection.terminal.component->bindTerminal(connection.terminal.index, view);
      }
      return ret;
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::distributeVectors()
    {
      for (auto& bus : buses_)
      {
        const auto range = bus->variableRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          bus->y()[static_cast<size_t>(i)]  = y_[range.begin + i];
          bus->yp()[static_cast<size_t>(i)] = yp_[range.begin + i];
        }
      }
      for (auto& component : components_)
      {
        const auto range = component->variableRange();
        for (IdxT i = 0; i < range.size; ++i)
        {
          component->y()[static_cast<size_t>(i)]  = y_[range.begin + i];
          component->yp()[static_cast<size_t>(i)] = yp_[range.begin + i];
        }
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::assembleTerminalCurrents(const ComponentT& component)
    {
      for (const auto& connection : connections_)
      {
        if (connection.terminal.component != &component)
        {
          continue;
        }

        const auto* current = component.terminalCurrent(connection.terminal.index);
        if (current == nullptr)
        {
          continue;
        }

        for (size_t phase = 0; phase < BusT::PHASE_COUNT; ++phase)
        {
          f_[static_cast<size_t>(connection.bus->residualIndex(phase))] -= current[phase];
        }
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::stampTerminalDifferentiability(const ComponentT& component)
    {
      for (const auto& connection : connections_)
      {
        if (connection.terminal.component != &component
            || !component.terminalHasDerivativeFeedthrough(connection.terminal.index))
        {
          continue;
        }

        for (size_t phase = 0; phase < BusT::PHASE_COUNT; ++phase)
        {
          tag_[static_cast<size_t>(connection.bus->variableIndex(phase))] = true;
        }
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::appendJacobianEntries(ComponentT& component)
    {
      auto  entries = component.getJacobian().getEntries(false);
      auto& rows    = std::get<0>(entries);
      auto& cols    = std::get<1>(entries);
      auto& vals    = std::get<2>(entries);
      if (!rows.empty())
      {
        J_.setValues(1.0, rows.data(), cols.data(), vals.data(), static_cast<IdxT>(rows.size()));
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::assembleCsrFromCurrentJacobians()
    {
      IdxT nnz_dup = 0;
      for (auto& bus : buses_)
      {
        auto entries  = bus->getJacobian().getEntries(false);
        nnz_dup      += static_cast<IdxT>(std::get<0>(entries).size());
      }
      for (auto& component : components_)
      {
        auto entries  = component->getJacobian().getEntries(false);
        nnz_dup      += static_cast<IdxT>(std::get<0>(entries).size());
      }

      auto* rows_dup = new IdxT[nnz_dup];
      auto* cols_dup = new IdxT[nnz_dup];
      auto* vals_dup = new RealT[nnz_dup];

      IdxT counter      = 0;
      auto copy_entries = [&](ComponentT& component)
      {
        auto  entries = component.getJacobian().getEntries(false);
        auto& rows    = std::get<0>(entries);
        auto& cols    = std::get<1>(entries);
        auto& vals    = std::get<2>(entries);
        for (size_t i = 0; i < rows.size(); ++i)
        {
          rows_dup[counter] = rows[i];
          cols_dup[counter] = cols[i];
          vals_dup[counter] = vals[i];
          ++counter;
        }
      };

      for (auto& bus : buses_)
      {
        copy_entries(*bus);
      }
      for (auto& component : components_)
      {
        copy_entries(*component);
      }

      LinearAlgebra::CooMatrix<RealT, IdxT> jac(size_,
                                                size_,
                                                nnz_dup,
                                                &rows_dup,
                                                &cols_dup,
                                                &vals_dup);

      IdxT* row_ptrs = jac.getCsrRowData();
      nnz_           = jac.getNnz();

      auto* cols = new IdxT[nnz_];
      auto* vals = new RealT[nnz_];

      std::copy(jac.getColData(), jac.getColData() + nnz_, cols);
      std::copy(jac.getValues(), jac.getValues() + nnz_, vals);

      csr_jac_ = std::make_unique<CsrMatrixT>(size_, size_, nnz_, &row_ptrs, &cols, &vals);

      const IdxT* map_to_sorted = jac.getMapToSorted();
      const IdxT* map_to_dedup  = jac.getMapToDeduplicated();

      map_to_csr_.resize(static_cast<size_t>(nnz_dup));
      for (IdxT i = 0; i < nnz_dup; ++i)
      {
        map_to_csr_[map_to_sorted[i]] = map_to_dedup[i];
      }
    }

    template <class ScalarT, typename IdxT>
    void SystemModel<ScalarT, IdxT>::updateCsrValuesFromCurrentJacobians()
    {
      RealT* vals = csr_jac_->getValues();
      for (IdxT i = 0; i < csr_jac_->getNnz(); ++i)
      {
        vals[i] = 0.0;
      }

      IdxT counter        = 0;
      auto update_entries = [&](ComponentT& component)
      {
        auto  entries = component.getJacobian().getEntries(false);
        auto& values  = std::get<2>(entries);
        for (size_t i = 0; i < values.size(); ++i)
        {
          vals[map_to_csr_[static_cast<size_t>(counter)]] += values[i];
          ++counter;
        }
      };

      for (auto& bus : buses_)
      {
        update_entries(*bus);
      }
      for (auto& component : components_)
      {
        update_entries(*component);
      }
    }

  } // namespace EMT
} // namespace GridKit
