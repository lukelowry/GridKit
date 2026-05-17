#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Case.hpp>
#include <GridKit/Model/EMT/IO/JsonSupport.hpp>
#include <GridKit/Model/EMT/IO/ParamReader.hpp>
#include <GridKit/Model/EMT/System/ComponentDescriptor.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Detail
    {
      template <class... Ts>
      consteval bool allDescribablePack(std::tuple<Ts...>*)
      {
        return (Describable<Ts> && ...);
      }

      template <class Components>
      consteval bool allDescribable()
      {
        return allDescribablePack(static_cast<Components*>(nullptr));
      }

      template <class... Ts, class Visitor>
      bool dispatchClassPack(std::tuple<Ts...>*,
                             std::string_view cls,
                             Visitor&&        visit)
      {
        return ((cls == ComponentDescriptor<Ts>::class_name
                     ? (visit.template operator()<Ts>(), true)
                     : false)
                || ...);
      }

      template <class Components, class Visitor>
      bool dispatchClass(std::string_view cls, Visitor&& visit)
      {
        return dispatchClassPack(static_cast<Components*>(nullptr),
                                 cls,
                                 std::forward<Visitor>(visit));
      }

      inline std::string quoted(std::string_view value)
      {
        return "'" + std::string(value) + "'";
      }

      inline bool isIdentifier(std::string_view value)
      {
        if (value.empty())
        {
          return false;
        }

        const auto first = static_cast<unsigned char>(value.front());
        if (!(std::isalpha(first) || value.front() == '_'))
        {
          return false;
        }

        for (const char ch : value.substr(1))
        {
          const auto c = static_cast<unsigned char>(ch);
          if (!(std::isalnum(c) || ch == '_'))
          {
            return false;
          }
        }
        return true;
      }

      inline void requireIdentifier(std::string_view value,
                                    std::string_view context)
      {
        if (!isIdentifier(value))
        {
          throwCase(context, "must be an identifier");
        }
      }

      inline const nlohmann::json& requireObjectField(const nlohmann::json& obj,
                                                      std::string_view      key,
                                                      std::string_view      entity)
      {
        if (!obj.is_object())
        {
          throwCase(entity, "expected object");
        }
        const auto it = obj.find(std::string(key));
        if (it == obj.end())
        {
          throwCase(fieldContext(entity, key), "required field missing");
        }
        const auto& value = *it;
        if (!value.is_object())
        {
          throwCase(fieldContext(entity, key), "expected object");
        }
        return value;
      }

      inline const nlohmann::json& requireArrayField(const nlohmann::json& obj,
                                                     std::string_view      key,
                                                     std::string_view      entity)
      {
        if (!obj.is_object())
        {
          throwCase(entity, "expected object");
        }
        const auto it = obj.find(std::string(key));
        if (it == obj.end())
        {
          throwCase(fieldContext(entity, key), "required field missing");
        }
        const auto& value = *it;
        if (!value.is_array())
        {
          throwCase(fieldContext(entity, key), "expected array");
        }
        return value;
      }

      template <class T>
      T optionalValue(const nlohmann::json& obj,
                      std::string_view      key,
                      T                     fallback,
                      std::string_view      entity)
      {
        if (!obj.is_object())
        {
          throwCase(entity, "expected object");
        }

        const auto it = obj.find(std::string(key));
        if (it == obj.end())
        {
          return fallback;
        }
        return readJsonValue<T>(*it, fieldContext(entity, key));
      }

      inline std::string requiredString(const nlohmann::json& obj,
                                        std::string_view      key,
                                        std::string_view      entity)
      {
        return require<std::string>(obj, key, entity);
      }

      template <class Names>
      std::optional<std::size_t> indexOf(const Names&     names,
                                         std::string_view name)
      {
        for (std::size_t i = 0; i < names.size(); ++i)
        {
          if (names[i] == name)
          {
            return i;
          }
        }
        return std::nullopt;
      }

      template <class Names>
      void rejectUnknownNamedKeys(const nlohmann::json& obj,
                                  const Names&          names,
                                  std::string_view      entity,
                                  std::string_view      noun)
      {
        for (const auto& item : obj.items())
        {
          requireIdentifier(item.key(), fieldContext(entity, item.key()));
          if (!indexOf(names, item.key()).has_value())
          {
            throw CaseError(std::string(entity) + ": unknown " + std::string(noun)
                            + " '" + item.key() + "'");
          }
        }
      }

      template <class Names>
      void requireEveryNamedKey(const nlohmann::json& obj,
                                const Names&          names,
                                std::string_view      entity,
                                std::string_view      noun)
      {
        rejectUnknownNamedKeys(obj, names, entity, noun);
        for (const auto name : names)
        {
          if (obj.find(std::string(name)) == obj.end())
          {
            throw CaseError(std::string(entity) + ": missing " + std::string(noun)
                            + " '" + std::string(name) + "'");
          }
        }
      }

      template <class Names>
      void appendPortNames(std::vector<std::string_view>& ports,
                           const Names&                   names)
      {
        for (const auto name : names)
        {
          ports.push_back(name);
        }
      }

      template <class ComponentT>
      std::vector<std::string_view> connectablePortNames()
      {
        std::vector<std::string_view> ports;
        appendPortNames(ports, ComponentDescriptor<ComponentT>::electrical_ports);
        appendPortNames(ports, ComponentDescriptor<ComponentT>::input_ports);
        return ports;
      }

      template <class ComponentT>
      void validatePortDescriptor(std::string_view entity)
      {
        std::vector<std::string_view> ports;
        appendPortNames(ports, ComponentDescriptor<ComponentT>::electrical_ports);
        appendPortNames(ports, ComponentDescriptor<ComponentT>::input_ports);
        appendPortNames(ports, ComponentDescriptor<ComponentT>::output_ports);

        for (std::size_t i = 0; i < ports.size(); ++i)
        {
          for (std::size_t j = i + 1; j < ports.size(); ++j)
          {
            if (ports[i] == ports[j])
            {
              throw CaseError(std::string(entity) + ": duplicate port '"
                              + std::string(ports[i]) + "' in class '"
                              + std::string(ComponentDescriptor<ComponentT>::class_name)
                              + "'");
            }
          }
        }
      }

      inline void registerUniqueName(auto&            table,
                                     std::string_view name,
                                     std::string_view entity)
      {
        requireIdentifier(name, fieldContext(entity, "name"));
        if (!table.names.insert(std::string(name)).second)
        {
          throw CaseError(std::string(entity) + ": name " + quoted(name)
                          + " already used");
        }
      }

      inline std::string lowerAscii(std::string value)
      {
        std::transform(value.begin(),
                       value.end(),
                       value.begin(),
                       [](unsigned char ch)
                       {
                         return static_cast<char>(std::tolower(ch));
                       });
        return value;
      }

      inline GridKit::Model::VariableMonitorFormat parseMonitorFormat(std::string      format,
                                                                      std::string_view entity)
      {
        format = lowerAscii(std::move(format));
        if (format == "csv")
        {
          return GridKit::Model::VariableMonitorFormat::CSV;
        }
        if (format == "json")
        {
          return GridKit::Model::VariableMonitorFormat::JSON;
        }
        if (format == "yaml")
        {
          return GridKit::Model::VariableMonitorFormat::YAML;
        }
        throw CaseError(std::string(entity) + ": unsupported monitor format '"
                        + format + "'");
      }

      inline std::string resolvePath(std::filesystem::path base,
                                     std::string           file)
      {
        std::filesystem::path path(file);
        if (!path.empty() && !path.is_absolute() && !base.empty())
        {
          path = std::move(base) / path;
        }
        return path.string();
      }

      template <class ComponentT>
      ComponentT constructFromParams(const nlohmann::json& params,
                                     const std::filesystem::path& base_dir,
                                     std::string_view      entity)
      {
        using Descriptor = ComponentDescriptor<ComponentT>;
        using Data       = typename Descriptor::Data;

        try
        {
          if constexpr (requires {
                          ComponentT::fromJson(params, base_dir, entity);
                        })
          {
            return ComponentT::fromJson(params, base_dir, entity);
          }
          else
          {
            return ComponentT(parseStruct<Data>(params, Descriptor::params, entity));
          }
        }
        catch (const CaseError&)
        {
          throw;
        }
        catch (const std::exception& ex)
        {
          throw CaseError(std::string(entity) + ": " + ex.what());
        }
      }

      inline std::vector<std::string> readMonList(const nlohmann::json& obj,
                                                  std::string_view      entity)
      {
        if (!obj.is_object())
        {
          throwCase(entity, "expected object");
        }

        const auto it = obj.find("mon");
        if (it == obj.end())
        {
          return {};
        }
        if (!it->is_array())
        {
          throwCase(fieldContext(entity, "mon"), "expected array");
        }

        std::vector<std::string> result;
        result.reserve(it->size());
        for (std::size_t i = 0; i < it->size(); ++i)
        {
          const auto context = std::string(entity) + ".mon[" + std::to_string(i) + "]";
          auto       value   = readJsonValue<std::string>((*it)[i], context);
          requireIdentifier(value, context);
          result.push_back(std::move(value));
        }
        return result;
      }

      inline std::vector<BusMonitorVariable> encodeBusMonitorVariables(
          const std::vector<std::string>& variables,
          std::string_view                bus_name,
          std::string_view                entity)
      {
        std::vector<BusMonitorVariable> encoded;
        encoded.reserve(variables.size());
        for (const auto& variable : variables)
        {
          const auto resolved = resolveBusMonitorVariable(variable);
          if (!resolved.has_value())
          {
            throw CaseError(std::string(entity) + ": monitor variable '" + variable
                            + "' is not valid for bus '" + std::string(bus_name) + "'");
          }
          encoded.push_back(*resolved);
        }
        return encoded;
      }

      template <class ComponentT>
      std::vector<std::size_t> encodeComponentMonitorVariables(
          const std::vector<std::string>& variables,
          std::string_view                entity)
      {
        std::vector<std::size_t> encoded;
        encoded.reserve(variables.size());

        if constexpr (requires(std::string_view name) {
                        ComponentMonitorTraits<ComponentT>::resolve(name);
                      })
        {
          for (const auto& variable : variables)
          {
            const auto resolved = ComponentMonitorTraits<ComponentT>::resolve(variable);
            if (!resolved.has_value())
            {
              throw CaseError(std::string(entity) + ": monitor variable '" + variable
                              + "' is not valid for class '"
                              + std::string(ComponentDescriptor<ComponentT>::class_name)
                              + "'");
            }
            encoded.push_back(static_cast<std::size_t>(*resolved));
          }
        }
        else
        {
          if (!variables.empty())
          {
            throw CaseError(std::string(entity) + ": class '"
                            + std::string(ComponentDescriptor<ComponentT>::class_name)
                            + "' does not define monitor variables");
          }
        }

        return encoded;
      }

      template <class DataT>
      struct AddComponentVisitor
      {
        DataT&                   data;
        const nlohmann::json&    params;
        const std::filesystem::path& base_dir;
        std::string              entity;
        std::string              label;
        std::vector<std::string> monitor_variables;
        ComponentRef             ref{};

        template <class ComponentT>
        void operator()()
        {
          auto       component = constructFromParams<ComponentT>(params,
                                                           base_dir,
                                                           entity + ": params");
          auto       monitors  = encodeComponentMonitorVariables<ComponentT>(monitor_variables,
                                                                      entity);
          const auto typed_ref = data.add(std::move(component));
          ref                  = ComponentRef{typed_ref.id};
          if (!monitors.empty())
          {
            data.component_monitors.push_back({ref, label, std::move(monitors)});
          }
        }
      };

      template <class DataT>
      void loadBuses(const nlohmann::json& root,
                     Case<DataT>&          loaded)
      {
        using RealT = typename DataT::scalar_type;
        using IdxT  = typename DataT::index_type;

        const auto& header = requireObjectField(root, "header", "case file");
        rejectUnknownKeys(header,
                          {"format_version", "case_name", "description"},
                          "header");

        const int version = require<int>(header, "format_version", "header");
        if (version != 1)
        {
          throw CaseError("header.format_version: unsupported version "
                          + std::to_string(version));
        }

        if (header.find("case_name") != header.end())
        {
          (void) require<std::string>(header, "case_name", "header");
        }
        if (header.find("description") != header.end())
        {
          (void) require<std::string>(header, "description", "header");
        }

        const auto& buses = requireArrayField(root, "buses", "case file");

        for (std::size_t i = 0; i < buses.size(); ++i)
        {
          const std::string entity = "bus[" + std::to_string(i) + "]";
          const auto&       entry  = buses[i];
          if (!entry.is_object())
          {
            throwCase(entity, "expected object");
          }
          rejectUnknownKeys(entry, {"name", "init", "mon"}, entity);

          const auto name = requiredString(entry, "name", entity);
          registerUniqueName(loaded.names, name, entity);

          const auto& init        = requireObjectField(entry, "init", entity);
          const auto  init_entity = fieldContext(entity, "init");
          rejectUnknownKeys(init, {"vm", "va"}, init_entity);

          BusData<RealT, IdxT> data{};
          data.vm = require<RealT>(init, "vm", init_entity);
          data.va = require<RealT>(init, "va", init_entity);

          const IdxT bus = loaded.data.addBus(data);
          loaded.names.buses.emplace(name, bus);

          auto monitors = encodeBusMonitorVariables(readMonList(entry, entity), name, entity);
          if (!monitors.empty())
          {
            loaded.data.bus_monitors.push_back({bus, name, std::move(monitors)});
          }
        }
      }

      template <class DataT>
      void loadComponents(const nlohmann::json& root,
                          Case<DataT>&          loaded,
                          const std::filesystem::path& base_dir)
      {
        const auto& components = requireArrayField(root, "components", "case file");
        for (std::size_t i = 0; i < components.size(); ++i)
        {
          const std::string entity = "component[" + std::to_string(i) + "]";
          const auto&       entry  = components[i];
          if (!entry.is_object())
          {
            throwCase(entity, "expected object");
          }
          rejectUnknownKeys(entry, {"name", "class", "params", "ports", "mon"}, entity);

          const auto name = requiredString(entry, "name", entity);
          registerUniqueName(loaded.names, name, entity);

          const auto  cls    = requiredString(entry, "class", entity);
          const auto& params = requireObjectField(entry, "params", entity);
          (void) requireObjectField(entry, "ports", entity);

          AddComponentVisitor<DataT> visitor{loaded.data,
                                             params,
                                             base_dir,
                                             "component '" + name + "'",
                                             name,
                                             readMonList(entry, entity)};
          const bool                 matched = dispatchClass<typename DataT::component_types>(cls, visitor);
          if (!matched)
          {
            throw CaseError("component '" + name + "': unknown class '" + cls + "'");
          }
          loaded.names.components.emplace(name, visitor.ref);
        }
      }

      inline std::pair<std::string, std::string> splitPortReference(std::string_view reference,
                                                                    std::string_view entity)
      {
        const auto first = reference.find('.');
        if (first == std::string_view::npos || first != reference.rfind('.')
            || first == 0 || first + 1 == reference.size())
        {
          throw CaseError(std::string(entity) + ": expected port reference '<producer>.<output_port>'");
        }

        auto producer = std::string(reference.substr(0, first));
        auto output   = std::string(reference.substr(first + 1));
        requireIdentifier(producer, entity);
        requireIdentifier(output, entity);
        return {std::move(producer), std::move(output)};
      }

      template <class DataT, class ComponentT>
      void connectPortsFor(DataT&                  data,
                           const CaseNames<DataT>& names,
                           ComponentRef            ref,
                           const nlohmann::json&   ports,
                           std::string_view        entity)
      {
        validatePortDescriptor<ComponentT>(entity);
        const auto required_ports = connectablePortNames<ComponentT>();
        requireEveryNamedKey(ports, required_ports, entity, "port");

        for (const auto& item : ports.items())
        {
          if (const auto electrical_port = indexOf(ComponentDescriptor<ComponentT>::electrical_ports,
                                                   item.key()))
          {
            const auto bus = readJsonValue<std::string>(item.value(),
                                                        fieldContext(entity, item.key()));
            requireIdentifier(bus, fieldContext(entity, item.key()));

            const auto bus_it = names.buses.find(bus);
            if (bus_it == names.buses.end())
            {
              throw CaseError(std::string(entity) + ": port '" + item.key()
                              + "' references unknown bus '" + bus + "'");
            }
            data.connect(ref.port(*electrical_port), bus_it->second);
            continue;
          }

          const auto input_port = indexOf(ComponentDescriptor<ComponentT>::input_ports,
                                          item.key());
          if (!input_port.has_value())
          {
            continue;
          }

          const auto reference = readJsonValue<std::string>(item.value(),
                                                            fieldContext(entity, item.key()));
          const auto [producer_name, output_name] =
              splitPortReference(reference, fieldContext(entity, item.key()));

          const auto producer_it = names.components.find(producer_name);
          if (producer_it == names.components.end())
          {
            throw CaseError(std::string(entity) + ": port '" + item.key()
                            + "' references unknown component '" + producer_name + "'");
          }

          std::optional<std::size_t> output_port;
          const bool                 producer_found = data.components.visit(
              producer_it->second.id,
              [&](const auto& producer)
              {
                using ProducerT = std::decay_t<decltype(producer)>;
                (void) producer;
                output_port = indexOf(ComponentDescriptor<ProducerT>::output_ports, output_name);
              });
          if (!producer_found)
          {
            throw CaseError(std::string(entity) + ": port '" + item.key()
                            + "' references missing producer '" + producer_name + "'");
          }
          if (!output_port.has_value())
          {
            throw CaseError(std::string(entity) + ": port '" + item.key()
                            + "' references unknown output port '" + reference + "'");
          }

          data.connect(producer_it->second.outputPort(*output_port),
                       ref.inputPort(*input_port));
        }
      }

      template <class DataT>
      void connectComponents(const nlohmann::json& root,
                             Case<DataT>&          loaded)
      {
        const auto& components = requireArrayField(root, "components", "case file");
        for (std::size_t i = 0; i < components.size(); ++i)
        {
          const auto& entry  = components[i];
          const auto  name   = requiredString(entry, "name", "component[" + std::to_string(i) + "]");
          const auto  entity = "component '" + name + "'";
          const auto& ports  = requireObjectField(entry, "ports", entity);

          const auto ref_it = loaded.names.components.find(name);
          if (ref_it == loaded.names.components.end())
          {
            throw CaseError(entity + ": component was not constructed");
          }

          const bool found = loaded.data.components.visit(
              ref_it->second.id,
              [&](const auto& component)
              {
                using ComponentT = std::decay_t<decltype(component)>;
                (void) component;
                connectPortsFor<DataT, ComponentT>(loaded.data,
                                                   loaded.names,
                                                   ref_it->second,
                                                   ports,
                                                   entity);
              });
          if (!found)
          {
            throw CaseError(entity + ": component was not constructed");
          }
        }
      }

      template <class DataT>
      void loadMonitorSinks(const nlohmann::json&        monitors,
                            Case<DataT>&                 loaded,
                            const std::filesystem::path& base_dir)
      {
        if (!monitors.is_array())
        {
          throwCase("monitors", "expected array");
        }

        for (std::size_t i = 0; i < monitors.size(); ++i)
        {
          const std::string entity = "monitors[" + std::to_string(i) + "]";
          const auto&       sink   = monitors[i];
          if (!sink.is_object())
          {
            throwCase(entity, "expected object");
          }
          rejectUnknownKeys(sink, {"file_name", "format", "delim"}, entity);

          auto file   = optionalValue<std::string>(sink, "file_name", std::string{}, entity);
          auto format = parseMonitorFormat(requiredString(sink, "format", entity),
                                           fieldContext(entity, "format"));
          auto delim  = optionalValue<std::string>(sink, "delim", ",", entity);

          loaded.data.addMonitorSink({resolvePath(base_dir, std::move(file)),
                                      format,
                                      std::move(delim)});
        }
      }

      template <class DataT>
      void loadMonitors(const nlohmann::json&        root,
                        Case<DataT>&                 loaded,
                        const std::filesystem::path& base_dir)
      {
        const auto it = root.find("monitors");
        if (it == root.end())
        {
          return;
        }
        loadMonitorSinks(*it, loaded, base_dir);
      }

      template <class DataT>
      Case<DataT> loadCaseImpl(const nlohmann::json&        root,
                               const std::filesystem::path& base_dir)
      {
        static_assert(allDescribable<typename DataT::component_types>(),
                      "loadCase<DataT>: every component type must have a complete ComponentDescriptor");

        if (!root.is_object())
        {
          throwCase("case file", "expected object");
        }
        rejectUnknownKeys(root, {"header", "buses", "components", "monitors"}, "case file");

        Case<DataT> loaded;
        loadBuses(root, loaded);
        loadComponents(root, loaded, base_dir);
        connectComponents(root, loaded);
        loadMonitors(root, loaded, base_dir);
        return loaded;
      }
    } // namespace Detail

    template <class DataT = CaseData<>>
    Case<DataT> loadCase(const nlohmann::json& root)
    {
      return Detail::loadCaseImpl<DataT>(root, std::filesystem::current_path());
    }

    template <class DataT = CaseData<>>
    Case<DataT> loadCase(const std::filesystem::path& path)
    {
      std::ifstream input(path);
      if (!input)
      {
        throw CaseError("case file '" + path.string() + "': not found");
      }

      nlohmann::json root;
      try
      {
        root = nlohmann::json::parse(input);
      }
      catch (const nlohmann::json::parse_error& ex)
      {
        throw CaseError("case file '" + path.string() + "': malformed JSON: "
                        + std::string(ex.what()));
      }

      return Detail::loadCaseImpl<DataT>(root, path.parent_path());
    }

    template <class DataT = CaseData<>>
    Case<DataT> loadCase(const char* path)
    {
      return loadCase<DataT>(std::filesystem::path{path});
    }

    template <class DataT = CaseData<>>
    Case<DataT> loadCase(const std::string& path)
    {
      return loadCase<DataT>(std::filesystem::path{path});
    }
  } // namespace EMT
} // namespace GridKit
