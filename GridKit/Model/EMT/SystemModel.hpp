#pragma once

#include <cassert>
#include <map>
#include <stdexcept>
#include <vector>

#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/VariableMonitorController.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class SystemModel : public EMT::Component<ScalarT, IdxT>
    {
      using bus_type       = EMT::Bus<ScalarT, IdxT>;
      using component_type = EMT::Component<ScalarT, IdxT>;
      using RealT          = typename Model::Evaluator<ScalarT, IdxT>::RealT;
      using CsrMatrixT     = typename Model::Evaluator<ScalarT, IdxT>::CsrMatrixT;

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
          auto* bus = new Bus<ScalarT, IdxT>(bus_data);
          addBus(bus);
        }

        for (const auto& component_data : data.branch_lumped_constant)
        {
          auto* branch = new BranchLumpedConstant<ScalarT, IdxT>(
              getBus(component_data.ports.at(BranchLumpedConstantData<RealT, IdxT>::Ports::from)),
              getBus(component_data.ports.at(BranchLumpedConstantData<RealT, IdxT>::Ports::to)),
              component_data);
          addComponent(branch);
        }

        for (const auto& component_data : data.load_rl)
        {
          auto* load = new LoadRL<ScalarT, IdxT>(
              getBus(component_data.ports.at(LoadRLData<RealT, IdxT>::Ports::ac)),
              component_data);
          addComponent(load);
        }

        for (const auto& component_data : data.voltage_source)
        {
          auto* source = new VoltageSource<ScalarT, IdxT>(
              getBus(component_data.ports.at(VoltageSourceData<RealT, IdxT>::Ports::bus)),
              component_data);
          addComponent(source);
        }

        for (const auto& component_data : data.breaker)
        {
          auto* breaker = new Breaker<ScalarT, IdxT>(
              getBus(component_data.ports.at(BreakerData<RealT, IdxT>::Ports::from)),
              getBus(component_data.ports.at(BreakerData<RealT, IdxT>::Ports::to)),
              component_data);
          addComponent(breaker);
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
          for (auto* component : components_)
          {
            delete component;
          }

          for (auto* bus : buses_)
          {
            delete bus;
          }
        }

        if (csr_jac_ != nullptr)
        {
          delete csr_jac_;
          csr_jac_ = nullptr;
        }
        if (map_to_csr_ != nullptr)
        {
          delete[] map_to_csr_;
          map_to_csr_ = nullptr;
        }
      }

      int setGridKitComponentID(IdxT component_id) override
      {
        gridkit_component_id_ = component_id;
        return 0;
      }

      int allocate() override
      {
        size_ = 0;

        for (auto* bus : buses_)
        {
          bus->allocate();
          for (IdxT j = 0; j < bus->size(); ++j)
          {
            bus->setVariableIndex(j, size_ + j);
            bus->setResidualIndex(j, size_ + j);
          }
          size_ += bus->size();
        }

        for (auto* component : components_)
        {
          component->allocate();
          for (IdxT j = 0; j < component->size(); ++j)
          {
            component->setVariableIndex(j, size_ + j);
            component->setResidualIndex(j, size_ + j);
          }
          size_ += component->size();
        }

        y_.resize(static_cast<size_t>(size_));
        yp_.resize(static_cast<size_t>(size_));
        f_.resize(static_cast<size_t>(size_));
        tag_.resize(static_cast<size_t>(size_));
        variable_indices_.resize(static_cast<size_t>(size_));
        residual_indices_.resize(static_cast<size_t>(size_));

        for (IdxT j = 0; j < size_; ++j)
        {
          this->setVariableIndex(j, j);
          this->setResidualIndex(j, j);
        }

        int error_count = this->verify();
        if (error_count > 0)
        {
          Log::error() << "EMT component errors: " << error_count << std::endl;
          throw std::runtime_error("EMT SystemModel allocation failed");
        }

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
        for (const auto* component : components_)
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
        for (auto* bus : buses_)
        {
          bus->initialize();
        }

        for (auto* bus : buses_)
        {
          for (IdxT j = 0; j < bus->size(); ++j)
          {
            y_[bus->getVariableIndex(j)]  = bus->y()[j];
            yp_[bus->getVariableIndex(j)] = bus->yp()[j];
          }
        }

        for (auto* component : components_)
        {
          component->initialize();
        }

        for (auto* component : components_)
        {
          for (IdxT j = 0; j < component->size(); ++j)
          {
            y_[component->getVariableIndex(j)]  = component->y()[j];
            yp_[component->getVariableIndex(j)] = component->yp()[j];
          }
        }

        return 0;
      }

      void initializeMonitor()
      {
        for (const auto* bus : buses_)
        {
          auto* mon = bus->getMonitor();
          if (mon && !mon->empty())
          {
            monitor_.addMonitor(mon);
          }
        }

        for (const auto* component : components_)
        {
          auto* mon = component->getMonitor();
          if (mon && !mon->empty())
          {
            monitor_.addMonitor(mon);
          }
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

      int evaluateResidual() override
      {
        updateVariables();

        for (auto* bus : buses_)
        {
          bus->evaluateResidual();
        }

        for (auto* component : components_)
        {
          component->evaluateResidual();
        }

        for (auto* bus : buses_)
        {
          for (IdxT j = 0; j < bus->size(); ++j)
          {
            f_[bus->getResidualIndex(j)] = bus->getResidual()[j];
          }
        }

        for (auto* component : components_)
        {
          for (IdxT j = 0; j < component->size(); ++j)
          {
            f_[component->getResidualIndex(j)] = component->getResidual()[j];
          }
        }

        return 0;
      }

      int evaluateJacobian() override
      {
        for (auto* bus : buses_)
        {
          bus->evaluateJacobian();
        }

        for (auto* component : components_)
        {
          component->evaluateJacobian();
        }

        nnz_ = 0;
        return 0;
      }

      CsrMatrixT* getCsrJacobian() const override
      {
        return csr_jac_;
      }

      void updateVariables()
      {
        for (auto* bus : buses_)
        {
          for (IdxT j = 0; j < bus->size(); ++j)
          {
            bus->y()[j]  = y_[bus->getVariableIndex(j)];
            bus->yp()[j] = yp_[bus->getVariableIndex(j)];
          }
        }

        for (auto* component : components_)
        {
          for (IdxT j = 0; j < component->size(); ++j)
          {
            component->y()[j]  = y_[component->getVariableIndex(j)];
            component->yp()[j] = yp_[component->getVariableIndex(j)];
          }
        }
      }

      void updateTime(RealT t, RealT a) override
      {
        time_  = t;
        alpha_ = a;
        for (auto* bus : buses_)
        {
          bus->updateTime(t, a);
        }
        for (auto* component : components_)
        {
          component->updateTime(t, a);
        }
        updateVariables();
      }

      void addBus(bus_type* bus)
      {
        IdxT gridkit_bus_id = static_cast<IdxT>(buses_.size());
        if (gridkit_bus_indices_.contains(bus->busID()))
        {
          bus->setBusID(gridkit_bus_id);
        }
        gridkit_bus_indices_[bus->busID()] = gridkit_bus_id;
        if (!bus->name().empty())
        {
          bus_name_to_index_[bus->name()] = gridkit_bus_id;
        }
        buses_.push_back(bus);
      }

      void addComponent(component_type* component)
      {
        IdxT gridkit_component_id = static_cast<IdxT>(components_.size());
        component->setGridKitComponentID(gridkit_component_id);
        components_.push_back(component);
      }

      bus_type* getBus(IdxT bus_id)
      {
        IdxT gridkit_bus_id = gridkit_bus_indices_.at(bus_id);
        assert((buses_[gridkit_bus_id])->busID() == bus_id);
        return buses_[gridkit_bus_id];
      }

      bus_type* getBusByName(const std::string& name)
      {
        return buses_[bus_name_to_index_.at(name)];
      }

      component_type* getComponent(IdxT gridkit_component_id)
      {
        return components_[gridkit_component_id];
      }

    private:
      std::vector<bus_type*>       buses_;
      std::vector<component_type*> components_;

      std::map<std::string, IdxT> bus_name_to_index_;
      std::map<IdxT, IdxT>        gridkit_bus_indices_;

      bool owns_components_{false};

      IdxT*       map_to_csr_{nullptr};
      CsrMatrixT* csr_jac_{nullptr};

      Model::VariableMonitorController<ScalarT> monitor_;
    };
  } // namespace EMT
} // namespace GridKit
