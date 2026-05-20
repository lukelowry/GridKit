#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/LinearAlgebra/MemoryUtils.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace fs = std::filesystem;

namespace
{
  using ScalarT = double;
  using RealT   = double;
  using IdxT    = std::size_t;
  using Json    = nlohmann::json;
  using Log     = GridKit::Utilities::Logger;

  struct Event
  {
    enum class Type
    {
      FaultOn,
      FaultOff
    };

    RealT time{0.0};
    Type  type{Type::FaultOn};
  };

  struct FaultSpec
  {
    std::string bus;
    RealT       conductance{0.0};
  };

  struct Study
  {
    fs::path           solver_file;
    fs::path           case_file;
    RealT              t_start{0.0};
    RealT              t_stop{10.0};
    RealT              output_step{1.0e-3};
    FaultSpec          fault;
    std::vector<Event> events;
  };

  fs::path resolvePath(const fs::path& base, const fs::path& path)
  {
    return path.is_absolute() ? path : base / path;
  }

  Event::Type parseEventType(const std::string& value)
  {
    if (value == "fault_on" || value == "FAULT_ON" || value == "FaultOn")
    {
      return Event::Type::FaultOn;
    }
    if (value == "fault_off" || value == "FAULT_OFF" || value == "FaultOff")
    {
      return Event::Type::FaultOff;
    }
    throw std::invalid_argument("Unsupported EMT event type: " + value);
  }

  Study parseStudy(const fs::path& solver_file)
  {
    std::ifstream input(solver_file);
    if (!input)
    {
      throw std::runtime_error("Could not open solver file: " + solver_file.string());
    }

    const auto json = Json::parse(input);
    Study      study;
    study.solver_file = solver_file;

    const auto base = solver_file.parent_path();
    const auto case_path =
        json.contains("case_file") ? json.at("case_file").get<fs::path>()
                                   : json.at("system_model_file").get<fs::path>();
    study.case_file = resolvePath(base, case_path);

    study.t_start     = json.value("t_start", study.t_start);
    study.t_stop      = json.value("t_stop", json.value("tmax", study.t_stop));
    study.output_step = json.value("output_step", json.value("dt", study.output_step));
    if (study.output_step <= 0.0)
    {
      throw std::invalid_argument("EMTSim output_step must be positive");
    }

    if (json.contains("fault"))
    {
      const auto& fault = json.at("fault");
      fault.at("bus").get_to(study.fault.bus);
      if (fault.contains("conductance"))
      {
        fault.at("conductance").get_to(study.fault.conductance);
      }
      else
      {
        const auto resistance = fault.at("resistance").get<RealT>();
        if (resistance <= 0.0)
        {
          throw std::invalid_argument("EMTSim fault resistance must be positive");
        }
        study.fault.conductance = 1.0 / resistance;
      }
    }

    if (json.contains("events"))
    {
      for (const auto& raw_event : json.at("events"))
      {
        Event event;
        raw_event.at("time").get_to(event.time);
        event.type = parseEventType(raw_event.at("type").get<std::string>());
        study.events.push_back(event);
      }
    }

    std::sort(study.events.begin(),
              study.events.end(),
              [](const Event& lhs, const Event& rhs)
              {
                return lhs.time < rhs.time;
              });

    return study;
  }

  class FaultedSystemModel final : public GridKit::EMT::SystemModel<ScalarT, IdxT>
  {
  public:
    using Base = GridKit::EMT::SystemModel<ScalarT, IdxT>;

    FaultedSystemModel(const GridKit::EMT::SystemModelData<double, IdxT>& data,
                       FaultSpec                                          fault)
      : Base(data),
        fault_(std::move(fault))
    {
    }

    int allocate() override
    {
      const int ret = Base::allocate();
      bindFaultBus();
      bindFaultJacobianSlots();
      return ret;
    }

    void setFaultActive(bool active)
    {
      fault_active_ = active;
    }

    void setMaxSteps(IdxT& max_steps) const override
    {
      max_steps = 500000;
    }

    int evaluateResidual() override
    {
      const int ret = Base::evaluateResidual();
      if (fault_active_ && fault_.conductance != 0.0)
      {
        auto& residual = this->getResidual();
        auto& state    = this->y();
        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const auto row    = static_cast<std::size_t>(fault_equation_start_ + phase);
          const auto col    = static_cast<std::size_t>(fault_variable_start_ + phase);
          residual.at(row) -= fault_.conductance * state.at(col);
        }
      }
      return ret;
    }

    int evaluateJacobian() override
    {
      const int ret = Base::evaluateJacobian();
      if (fault_active_ && fault_.conductance != 0.0)
      {
        auto* csr = this->getCsrJacobian();
        auto* val = csr->getValues(GridKit::LinearAlgebra::memory::HOST);
        for (const auto slot : fault_jacobian_slots_)
        {
          val[slot] -= fault_.conductance;
        }
        csr->setUpdated(GridKit::LinearAlgebra::memory::HOST);
      }
      return ret;
    }

  private:
    void bindFaultBus()
    {
      if (fault_.bus.empty() || fault_.conductance == 0.0)
      {
        return;
      }

      for (const auto& bus : this->layout().buses)
      {
        if (bus.name == fault_.bus)
        {
          fault_variable_start_ = bus.variable_start;
          fault_equation_start_ = bus.equation_start;
          fault_bound_          = true;
          return;
        }
      }

      throw std::runtime_error("EMTSim fault bus not found: " + fault_.bus);
    }

    void bindFaultJacobianSlots()
    {
      fault_jacobian_slots_.clear();
      if (!fault_bound_ || fault_.conductance == 0.0)
      {
        return;
      }

      auto* csr  = this->getCsrJacobian();
      auto* rows = csr->getRowData(GridKit::LinearAlgebra::memory::HOST);
      auto* cols = csr->getColData(GridKit::LinearAlgebra::memory::HOST);

      for (IdxT phase = 0; phase < 3; ++phase)
      {
        const auto row = fault_equation_start_ + phase;
        const auto col = fault_variable_start_ + phase;
        bool       hit = false;
        for (IdxT slot = rows[row]; slot < rows[row + 1]; ++slot)
        {
          if (cols[slot] == col)
          {
            fault_jacobian_slots_.push_back(slot);
            hit = true;
            break;
          }
        }

        if (!hit)
        {
          throw std::runtime_error("EMTSim fault requires an existing bus-voltage diagonal Jacobian slot");
        }
      }
    }

    FaultSpec         fault_;
    bool              fault_active_{false};
    bool              fault_bound_{false};
    IdxT              fault_variable_start_{0};
    IdxT              fault_equation_start_{0};
    std::vector<IdxT> fault_jacobian_slots_;
  };

  void runSegment(AnalysisManager::Sundials::Ida<ScalarT, IdxT>& ida,
                  RealT&                                         current_time,
                  RealT                                          stop_time,
                  RealT                                          output_step)
  {
    if (stop_time <= current_time)
    {
      return;
    }

    const auto nout = std::max(1, static_cast<int>(std::llround((stop_time - current_time) / output_step)));
    ida.runSimulation(stop_time, nout);
    current_time = stop_time;
  }
} // namespace

int main(int argc, const char* argv[])
{
  if (argc < 2)
  {
    Log::error() << "No input file provided\n"
                 << "Usage: EMTSim <solver-json>\n";
    return 1;
  }

  try
  {
    const Study study = parseStudy(fs::absolute(argv[1]));
    auto        data  = GridKit::EMT::parseSystemModelData(study.case_file);

    FaultedSystemModel system(data, study.fault);
    system.allocate();

    AnalysisManager::Sundials::Ida<ScalarT, IdxT> ida(&system);
    ida.configureSimulation();
    ida.initializeSimulation(study.t_start, true);

    RealT current_time = study.t_start;
    for (const auto& event : study.events)
    {
      runSegment(ida, current_time, event.time, study.output_step);

      if (event.type == Event::Type::FaultOn)
      {
        system.setFaultActive(true);
      }
      else
      {
        system.setFaultActive(false);
      }

      ida.initializeSimulation(event.time, true);
      current_time = event.time;
    }

    runSegment(ida, current_time, study.t_stop, study.output_step);
    system.stopMonitor();

    std::cout << "EMTSim complete: " << study.case_file << '\n';
    return 0;
  }
  catch (const std::exception& exc)
  {
    Log::error() << exc.what() << std::endl;
    return 1;
  }
}
