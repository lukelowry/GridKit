/**
 * @file CaseData.cpp
 * @brief Shared JSON case schema parser.
 */

#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <GridKit/Model/Case/CaseData.hpp>

namespace GridKit
{
  namespace Model
  {
    namespace Case
    {
      namespace
      {
        using FieldSet = std::set<std::string_view>;

        std::runtime_error error(const std::string& path, const std::string& message)
        {
          return std::runtime_error("Case parser: " + path + ": " + message);
        }

        void requireObject(const Json& object, const std::string& path)
        {
          if (!object.is_object())
          {
            throw error(path, "expected an object");
          }
        }

        void requireArray(const Json& object, const std::string& path)
        {
          if (!object.is_array())
          {
            throw error(path, "expected an array");
          }
        }

        void checkKnownFields(const Json& object, const FieldSet& fields, const std::string& path)
        {
          requireObject(object, path);
          for (const auto& item : object.items())
          {
            if (!fields.contains(std::string_view(item.key())))
            {
              throw error(path, "unknown field '" + item.key() + "'");
            }
          }
        }

        const Json& requireField(const Json&        object,
                                 const std::string& key,
                                 const std::string& path)
        {
          if (!object.contains(key))
          {
            throw error(path, "missing required field '" + key + "'");
          }
          return object.at(key);
        }

        Json objectField(const Json& object, const std::string& key, const std::string& path)
        {
          if (!object.contains(key))
          {
            return Json::object();
          }
          const auto& value = object.at(key);
          if (!value.is_object())
          {
            throw error(path + "." + key, "expected an object");
          }
          return value;
        }

        Json arrayField(const Json& object, const std::string& key, const std::string& path)
        {
          if (!object.contains(key))
          {
            return Json::array();
          }
          const auto& value = object.at(key);
          if (!value.is_array())
          {
            throw error(path + "." + key, "expected an array");
          }
          return value;
        }

        HeaderData parseHeader(const Json& input)
        {
          static const FieldSet fields{"format_version",
                                       "format_revision",
                                       "case_name",
                                       "case_date_time",
                                       "case_description",
                                       "case_comments",
                                       "freq_base",
                                       "va_base",
                                       "extension"};
          checkKnownFields(input, fields, "header");

          HeaderData data;
          data.format_version =
              requireField(input, "format_version", "header").get<unsigned short>();
          data.format_revision =
              requireField(input, "format_revision", "header").get<unsigned short>();
          data.case_name = requireField(input, "case_name", "header").get<std::string>();
          data.case_description =
              requireField(input, "case_description", "header").get<std::string>();
          data.case_comments = requireField(input, "case_comments", "header").get<std::string>();
          data.freq_base     = requireField(input, "freq_base", "header").get<double>();
          data.va_base       = requireField(input, "va_base", "header").get<double>();

          if (input.contains("case_date_time"))
          {
            data.case_date_time = input.at("case_date_time").get<std::string>();
          }
          data.extension = objectField(input, "extension", "header");
          return data;
        }

        ComponentData parseComponent(const Json&        input,
                                     const std::string& path,
                                     bool               bus_component)
        {
          static const FieldSet bus_fields{"number",
                                           "class",
                                           "name",
                                           "params",
                                           "init",
                                           "mon",
                                           "extension"};
          static const FieldSet component_fields{"id",
                                                 "class",
                                                 "name",
                                                 "params",
                                                 "init",
                                                 "ports",
                                                 "mon",
                                                 "extension"};

          checkKnownFields(input, bus_component ? bus_fields : component_fields, path);

          ComponentData data;
          if (bus_component)
          {
            data.number     = requireField(input, "number", path).get<long long>();
            data.has_number = true;
          }
          else
          {
            data.id = requireField(input, "id", path).get<std::string>();
          }

          data.component_class = requireField(input, "class", path).get<std::string>();
          if (input.contains("name"))
          {
            data.name = input.at("name").get<std::string>();
          }
          data.params    = objectField(input, "params", path);
          data.init      = objectField(input, "init", path);
          data.ports     = objectField(input, "ports", path);
          data.mon       = arrayField(input, "mon", path);
          data.extension = objectField(input, "extension", path);
          return data;
        }

        SignalData parseSignal(const Json& input, const std::string& path)
        {
          static const FieldSet fields{"signal_id", "name", "extension"};
          checkKnownFields(input, fields, path);

          SignalData data;
          data.signal_id = requireField(input, "signal_id", path).get<long long>();
          if (input.contains("name"))
          {
            data.name = input.at("name").get<std::string>();
          }
          data.extension = objectField(input, "extension", path);
          return data;
        }

        MonitorData parseMonitor(const Json& input, const std::string& path)
        {
          static const FieldSet fields{"file_name", "format", "delim", "extension"};
          checkKnownFields(input, fields, path);

          MonitorData data;
          if (input.contains("file_name"))
          {
            data.file_name = input.at("file_name").get<std::string>();
          }
          data.format = requireField(input, "format", path).get<std::string>();
          if (input.contains("delim"))
          {
            data.delim = input.at("delim").get<std::string>();
          }
          data.extension = objectField(input, "extension", path);
          return data;
        }

        std::vector<ComponentData> parseComponentArray(const Json&        input,
                                                       const std::string& path,
                                                       bool               bus_component)
        {
          requireArray(input, path);
          std::vector<ComponentData> data;
          data.reserve(input.size());
          for (size_t i = 0; i < input.size(); ++i)
          {
            data.push_back(parseComponent(input.at(i), path + "[" + std::to_string(i) + "]", bus_component));
          }
          return data;
        }

        std::vector<SignalData> parseSignalArray(const Json& input, const std::string& path)
        {
          requireArray(input, path);
          std::vector<SignalData> data;
          data.reserve(input.size());
          for (size_t i = 0; i < input.size(); ++i)
          {
            data.push_back(parseSignal(input.at(i), path + "[" + std::to_string(i) + "]"));
          }
          return data;
        }

        std::vector<MonitorData> parseMonitorArray(const Json& input, const std::string& path)
        {
          requireArray(input, path);
          std::vector<MonitorData> data;
          data.reserve(input.size());
          for (size_t i = 0; i < input.size(); ++i)
          {
            data.push_back(parseMonitor(input.at(i), path + "[" + std::to_string(i) + "]"));
          }
          return data;
        }

        template <typename ValueT, typename LabelT>
        void checkDuplicate(const ValueT&      value,
                            std::set<ValueT>&  seen,
                            const std::string& path,
                            LabelT             label)
        {
          if (!seen.insert(value).second)
          {
            throw error(path, "duplicate " + std::string(label));
          }
        }

        void validateDuplicates(const CaseData& data)
        {
          std::set<long long> bus_numbers;
          for (const auto& bus : data.buses)
          {
            checkDuplicate(bus.number, bus_numbers, "buses", "bus number");
          }

          std::set<std::string> branch_ids;
          for (const auto& branch : data.branches)
          {
            checkDuplicate(branch.id, branch_ids, "branches", std::string("branch id '") + branch.id + "'");
          }

          std::set<std::string> device_ids;
          for (const auto& device : data.devices)
          {
            checkDuplicate(device.id, device_ids, "devices", std::string("device id '") + device.id + "'");
          }

          std::set<long long> signal_ids;
          for (const auto& signal : data.signals)
          {
            checkDuplicate(signal.signal_id, signal_ids, "signals", "signal id");
          }

          for (const auto& library : data.models.items())
          {
            const auto& entries = library.value();
            if (!entries.is_array())
            {
              throw error("models." + library.key(), "expected an array");
            }

            std::set<std::string> model_ids;
            for (size_t i = 0; i < entries.size(); ++i)
            {
              const auto& entry = entries.at(i);
              if (!entry.is_object())
              {
                throw error("models." + library.key() + "[" + std::to_string(i) + "]",
                            "expected an object");
              }
              if (entry.contains("id"))
              {
                const auto id = entry.at("id").get<std::string>();
                checkDuplicate(id, model_ids, "models." + library.key(), std::string("model id '") + id + "'");
              }
            }
          }
        }

      } // namespace

      CaseData parseCaseData(std::istream& input)
      {
        Json root;
        input >> root;

        static const FieldSet fields{"header",
                                     "buses",
                                     "branches",
                                     "devices",
                                     "signals",
                                     "monitors",
                                     "models",
                                     "extension"};
        checkKnownFields(root, fields, "$");

        CaseData data;
        data.header   = parseHeader(requireField(root, "header", "$"));
        data.buses    = parseComponentArray(requireField(root, "buses", "$"), "buses", true);
        data.branches = parseComponentArray(requireField(root, "branches", "$"), "branches", false);
        data.devices  = parseComponentArray(requireField(root, "devices", "$"), "devices", false);
        data.signals  = parseSignalArray(requireField(root, "signals", "$"), "signals");
        if (root.contains("monitors"))
        {
          data.monitors = parseMonitorArray(root.at("monitors"), "monitors");
        }
        data.models    = root.contains("models") ? objectField(root, "models", "$") : Json::object();
        data.extension = objectField(root, "extension", "$");

        validateDuplicates(data);
        return data;
      }

      CaseData parseCaseData(const std::filesystem::path& input_file)
      {
        std::ifstream input(input_file);
        if (!input.good())
        {
          throw std::runtime_error("Cannot open file '" + input_file.string() + "'");
        }
        return parseCaseData(input);
      }

      CaseData parseCaseDataFromString(const std::string& input)
      {
        std::istringstream stream(input);
        return parseCaseData(stream);
      }

    } // namespace Case
  } // namespace Model
} // namespace GridKit
