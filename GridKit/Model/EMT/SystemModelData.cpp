/**
 * @file SystemModelData.cpp
 * @brief EMT JSON parser entry points.
 */

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <GridKit/Model/Case/CaseData.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Model/EMT/SystemModelDataJSONParser.hpp>

namespace GridKit
{
  namespace EMT
  {
    SystemModelData<double, size_t> parseSystemModelData(std::istream& input)
    {
      const auto case_data = Model::Case::parseCaseData(input);
      return systemModelDataFromCase<double, size_t>(case_data);
    }

    SystemModelData<double, size_t>
    parseSystemModelData(const std::filesystem::path& input_file)
    {
      std::ifstream input(input_file);
      if (!input.good())
      {
        throw std::runtime_error("Cannot open file '" + input_file.string() + "'");
      }
      return parseSystemModelData(input);
    }

    SystemModelData<double, size_t> parseSystemModelDataFromString(const std::string& input)
    {
      std::istringstream stream(input);
      return parseSystemModelData(stream);
    }

  } // namespace EMT
} // namespace GridKit
