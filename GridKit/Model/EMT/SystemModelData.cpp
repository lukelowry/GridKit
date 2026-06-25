#include "SystemModelData.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/SystemModelDataJSONParser.hpp>

namespace GridKit
{
  namespace EMT
  {
    SystemModelData<double, size_t, 3> parseSystemModelData(std::istream& stream)
    {
      return nlohmann::json::parse(stream).get<SystemModelData<double, size_t, 3>>();
    }

    SystemModelData<double, size_t, 3> parseSystemModelData(std::istream&& stream)
    {
      return parseSystemModelData(stream);
    }

    SystemModelData<double, size_t, 3> parseSystemModelData(const std::filesystem::path& path)
    {
      auto stream = std::ifstream(path);
      if (!stream)
      {
        std::stringstream ss;
        ss << "Could not open file: " << path;
        throw std::runtime_error(ss.str());
      }

      return parseSystemModelData(stream);
    }

    SystemModelData<double, size_t, 3> parseSystemModelData(const std::string& path)
    {
      return parseSystemModelData(std::filesystem::path{path});
    }
  } // namespace EMT
} // namespace GridKit
