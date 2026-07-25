#include "SystemModelData.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "SystemModelDataJSONParser.hpp"

namespace GridKit::EMT
{
  SystemModelData<double, std::size_t> parseSystemModelData(
      std::istream& stream)
  {
    return json::parse(stream).get<SystemModelData<double, std::size_t>>();
  }

  SystemModelData<double, std::size_t> parseSystemModelData(
      const std::filesystem::path& file_path)
  {
    std::ifstream stream(file_path);
    if (!stream)
    {
      throw std::runtime_error("Could not open EMT case: "
                               + file_path.string());
    }
    return parseSystemModelData(stream);
  }

} // namespace GridKit::EMT
