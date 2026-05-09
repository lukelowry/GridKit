/**
 * @file CaseData.hpp
 * @brief Shared JSON case schema data for GridKit model adapters.
 */

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace GridKit
{
  namespace Model
  {
    namespace Case
    {
      using Json = nlohmann::json;

      struct HeaderData
      {
        std::optional<unsigned short> format_version;
        std::optional<unsigned short> format_revision;
        std::string                   case_name;
        std::string                   case_date_time;
        std::string                   case_description;
        std::string                   case_comments;
        double                        freq_base{60.0};
        double                        va_base{100.0e6};
        Json                          extension{Json::object()};
      };

      struct ComponentData
      {
        std::string id;
        long long   number{0};
        bool        has_number{false};
        std::string component_class;
        std::string name;
        Json        params{Json::object()};
        Json        init{Json::object()};
        Json        ports{Json::object()};
        Json        mon{Json::array()};
        Json        extension{Json::object()};
      };

      struct SignalData
      {
        long long   signal_id{0};
        std::string name;
        Json        extension{Json::object()};
      };

      struct MonitorData
      {
        std::string file_name;
        std::string format;
        std::string delim{","};
        Json        extension{Json::object()};
      };

      struct CaseData
      {
        HeaderData                 header;
        std::vector<ComponentData> buses;
        std::vector<ComponentData> branches;
        std::vector<ComponentData> devices;
        std::vector<SignalData>    signals;
        std::vector<MonitorData>   monitors;
        Json                       models{Json::object()};
        Json                       extension{Json::object()};
      };

      CaseData parseCaseData(std::istream& input);
      CaseData parseCaseData(const std::filesystem::path& input_file);
      CaseData parseCaseDataFromString(const std::string& input);

    } // namespace Case
  } // namespace Model
} // namespace GridKit
