#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Model/EMT/InitialStateLayout.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>

namespace GridKit::EMT
{
  struct InitialStateVector
  {
    std::string                variable;
    std::optional<std::size_t> index;
    ABCVector<double>          value{};
  };

  struct BusInitialState
  {
    std::size_t                     bus_id{0};
    std::vector<InitialStateVector> states;
  };

  struct ComponentInitialState
  {
    std::string                     component_id;
    std::vector<InitialStateVector> states;
  };

  struct InitialStateData
  {
    std::string                        case_name;
    double                             time{0.0};
    std::vector<BusInitialState>       buses;
    std::vector<ComponentInitialState> components;
  };

  namespace InitialStateDetail
  {
    using json        = nlohmann::json;
    using VariableKey = std::pair<std::string, std::optional<std::size_t>>;

    struct InitialStateAssignment
    {
      std::size_t global_index;
      double      value;
    };

    inline void validateKeys(const json&                  object,
                             const std::set<std::string>& allowed,
                             const std::set<std::string>& required,
                             const std::string&           context)
    {
      if (!object.is_object())
      {
        throw std::runtime_error(context + " must be a JSON object");
      }
      for (const auto& [key, value] : object.items())
      {
        static_cast<void>(value);
        if (!allowed.contains(key))
        {
          throw std::runtime_error("Unknown " + context + " key: " + key);
        }
      }
      for (const auto& key : required)
      {
        if (!object.contains(key))
        {
          throw std::runtime_error("Missing " + context + " key: " + key);
        }
      }
    }

    inline std::size_t nonnegativeInteger(const json&        value,
                                          const std::string& context)
    {
      if (!value.is_number_integer() && !value.is_number_unsigned())
      {
        throw std::runtime_error(context + " must be a nonnegative integer");
      }

      const auto parsed = value.get<std::int64_t>();
      if (parsed < 0)
      {
        throw std::runtime_error(context + " must be a nonnegative integer");
      }
      return static_cast<std::size_t>(parsed);
    }

    inline ABCVector<double> abcVector(const json&        value,
                                       const std::string& context)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::runtime_error(context + " must contain exactly 3 values");
      }

      ABCVector<double> result{};
      for (std::size_t phase = 0; phase < result.size(); ++phase)
      {
        if (!value[phase].is_number())
        {
          throw std::runtime_error(context + " values must be numeric");
        }
        result[phase] = value[phase].get<double>();
        if (!std::isfinite(result[phase]))
        {
          throw std::runtime_error(context + " values must be finite");
        }
      }
      return result;
    }

    inline bool isDirectABCVector(const json& value)
    {
      if (!value.is_array() || value.size() != 3)
      {
        return false;
      }
      for (const auto& entry : value)
      {
        if (!entry.is_number())
        {
          return false;
        }
      }
      return true;
    }

    inline std::vector<InitialStateVector> parseVariables(
        const json&        variables,
        const std::string& context)
    {
      if (!variables.is_object() || variables.empty())
      {
        throw std::runtime_error(context + " variables must be a nonempty JSON object");
      }

      std::vector<InitialStateVector> result;
      std::set<VariableKey>           found;
      for (const auto& [name, encoded] : variables.items())
      {
        if (name.empty())
        {
          throw std::runtime_error(context + " variable names must be nonempty");
        }

        if (isDirectABCVector(encoded))
        {
          const VariableKey key{name, std::nullopt};
          found.insert(key);
          result.push_back(
              {name, std::nullopt, abcVector(encoded, context + " " + name)});
          continue;
        }

        if (!encoded.is_array() || encoded.empty())
        {
          throw std::runtime_error(
              context + " " + name + " must be an ABC vector or a nonempty indexed array");
        }
        for (const auto& entry : encoded)
        {
          validateKeys(entry,
                       {"index", "value"},
                       {"index", "value"},
                       context + " " + name + " state");
          const auto index = nonnegativeInteger(
              entry.at("index"), context + " " + name + " index");
          const VariableKey key{name, index};
          if (!found.insert(key).second)
          {
            throw std::runtime_error(
                "Duplicate " + context + " " + name + " index "
                + std::to_string(index));
          }
          result.push_back(
              {name,
               index,
               abcVector(entry.at("value"), context + " " + name)});
        }
      }
      return result;
    }

    inline std::string stateLabel(const InitialStateVector& state)
    {
      auto label = state.variable;
      if (state.index.has_value())
      {
        label += "[" + std::to_string(*state.index) + "]";
      }
      return label;
    }

    inline const InitialStateVariable& findVariable(
        const std::vector<InitialStateVariable>& layout,
        const InitialStateVector&                state,
        const std::string&                       context)
    {
      for (const auto& variable : layout)
      {
        if (variable.name == state.variable && variable.index == state.index)
        {
          return variable;
        }
      }
      throw std::runtime_error(
          context + " has no differential variable " + stateLabel(state));
    }

    template <typename OwnerT>
    void collectOwnerStates(
        OwnerT*                                owner,
        const std::vector<InitialStateVector>& states,
        const std::string&                     context,
        std::set<std::size_t>&                 assigned,
        std::vector<InitialStateAssignment>&   assignments)
    {
      if (owner == nullptr)
      {
        throw std::runtime_error(context + " does not exist");
      }
      const auto* layout_owner = dynamic_cast<const InitialStateLayout*>(owner);
      if (layout_owner == nullptr)
      {
        throw std::runtime_error(context + " has no external initial-state variables");
      }

      std::vector<InitialStateVariable> layout;
      layout_owner->appendInitialStateVariables(layout);
      for (const auto& state : states)
      {
        const auto& variable = findVariable(layout, state, context);
        if (variable.local_offset + state.value.size() > owner->size())
        {
          throw std::runtime_error(context + " initial-state layout is invalid");
        }

        for (std::size_t phase = 0; phase < state.value.size(); ++phase)
        {
          if (!std::isfinite(state.value[phase]))
          {
            throw std::runtime_error(
                context + " " + stateLabel(state) + " is non-finite");
          }
          const auto local_index = variable.local_offset + phase;
          if (!owner->tag().at(local_index))
          {
            throw std::runtime_error(
                context + " " + stateLabel(state) + " is algebraic");
          }
          const auto global_index = owner->getVariableIndex(local_index);
          if (!assigned.insert(global_index).second)
          {
            throw std::runtime_error(
                context + " assigns a differential variable more than once");
          }
          assignments.push_back({global_index, state.value[phase]});
        }
      }
    }
  } // namespace InitialStateDetail

  inline InitialStateData parseInitialStateData(
      const std::filesystem::path& file_path,
      const std::string&           expected_case_name)
  {
    using namespace InitialStateDetail;

    std::ifstream input(file_path);
    if (!input)
    {
      throw std::runtime_error("Could not open EMT initial-state file: "
                               + file_path.string());
    }

    const auto document = json::parse(input);
    validateKeys(document,
                 {"format_version",
                  "format_revision",
                  "case_name",
                  "time",
                  "buses",
                  "components"},
                 {"format_version",
                  "format_revision",
                  "case_name",
                  "time",
                  "buses",
                  "components"},
                 "EMT initial state");
    if (nonnegativeInteger(document.at("format_version"),
                           "EMT initial-state format_version")
            != 0
        || nonnegativeInteger(document.at("format_revision"),
                              "EMT initial-state format_revision")
               != 1)
    {
      throw std::runtime_error("Unsupported EMT initial-state format");
    }
    if (!document.at("case_name").is_string()
        || document.at("case_name").get<std::string>().empty())
    {
      throw std::runtime_error("EMT initial-state case_name must be nonempty");
    }
    const auto case_name = document.at("case_name").get<std::string>();
    if (case_name != expected_case_name)
    {
      throw std::runtime_error(
          "EMT initial-state case_name does not match the system model");
    }
    if (!document.at("time").is_number())
    {
      throw std::runtime_error("EMT initial-state time must be numeric");
    }
    const auto initial_time = document.at("time").get<double>();
    if (!std::isfinite(initial_time))
    {
      throw std::runtime_error("EMT initial-state time must be finite");
    }
    if (!document.at("buses").is_array()
        || !document.at("components").is_array())
    {
      throw std::runtime_error(
          "EMT initial-state buses and components must be arrays");
    }

    InitialStateData      result{case_name, initial_time, {}, {}};
    std::set<std::size_t> bus_ids;
    for (const auto& entry : document.at("buses"))
    {
      validateKeys(entry,
                   {"id", "variables"},
                   {"id", "variables"},
                   "EMT bus initial state");
      const auto bus_id = nonnegativeInteger(entry.at("id"),
                                             "EMT initial-state bus id");
      if (!bus_ids.insert(bus_id).second)
      {
        throw std::runtime_error("Duplicate EMT initial-state bus id");
      }
      result.buses.push_back(
          {bus_id,
           parseVariables(entry.at("variables"),
                          "EMT bus " + std::to_string(bus_id))});
    }

    std::set<std::string> component_ids;
    for (const auto& entry : document.at("components"))
    {
      validateKeys(entry,
                   {"id", "variables"},
                   {"id", "variables"},
                   "EMT component initial state");
      if (!entry.at("id").is_string()
          || entry.at("id").get<std::string>().empty())
      {
        throw std::runtime_error(
            "EMT initial-state component id must be nonempty");
      }

      const auto id = entry.at("id").get<std::string>();
      if (!component_ids.insert(id).second)
      {
        throw std::runtime_error(
            "Duplicate EMT initial-state component id: " + id);
      }
      result.components.push_back(
          {id,
           parseVariables(entry.at("variables"), "EMT component " + id)});
    }
    return result;
  }

  inline void applyInitialState(
      SystemModel<double, std::size_t>& system,
      const InitialStateData&           initial_state)
  {
    using namespace InitialStateDetail;

    if (system.tagDifferentiable() != 0)
    {
      throw std::runtime_error("Could not classify EMT initial states");
    }

    std::set<std::size_t>               assigned;
    std::vector<InitialStateAssignment> assignments;
    for (const auto& bus_state : initial_state.buses)
    {
      collectOwnerStates(system.getBus(bus_state.bus_id),
                         bus_state.states,
                         "EMT bus " + std::to_string(bus_state.bus_id),
                         assigned,
                         assignments);
    }
    for (const auto& component_state : initial_state.components)
    {
      collectOwnerStates(system.getComponent(component_state.component_id),
                         component_state.states,
                         "EMT component " + component_state.component_id,
                         assigned,
                         assignments);
    }

    const auto& differential = system.tag();
    for (std::size_t index = 0; index < differential.size(); ++index)
    {
      if (differential[index] && !assigned.contains(index))
      {
        throw std::runtime_error(
            "EMT initial state does not supply every differential variable");
      }
    }

    auto* values = system.y().getData();
    for (const auto& assignment : assignments)
    {
      values[assignment.global_index] = assignment.value;
    }
    system.y().setDataUpdated();
  }
} // namespace GridKit::EMT
