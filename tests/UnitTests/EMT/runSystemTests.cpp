#include <algorithm>
#include <array>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Testing/Testing.hpp>

#include "AnalysisUtilities.hpp"
#include "EMTTestFixture.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;

  using Json = nlohmann::json;
  using ConstantSourceT =
      PhasorDynamics::ConstantSignalSource<double, std::size_t>;
  using VoltageSourceT = EMT::VoltageSource<double, std::size_t>;
  using LineLumpedT    = EMT::LineLumped<double, std::size_t>;
  using LoadZT         = EMT::LoadZ<double, std::size_t>;
  using SwitchT        = EMT::Switch<double, std::size_t>;
  using VectorFitT     = EMT::VectorFit<double, std::size_t>;

  Json loadFixtureJson()
  {
    std::ifstream input(EMT_TEST_FIXTURE);
    return Json::parse(input);
  }

  std::string loadFixtureText()
  {
    std::ifstream     input(EMT_TEST_FIXTURE);
    std::stringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
  }

  Json& findDevice(Json& input, const std::string& id)
  {
    for (auto& device : input.at("devices"))
    {
      if (device.at("id").get<std::string>() == id)
      {
        return device;
      }
    }
    throw std::logic_error("EMT test fixture device not found: " + id);
  }

  Json zeroComplexMatrix()
  {
    const Json zero = Json::array({0.0, 0.0});
    return Json::array({Json::array({zero, zero, zero}),
                        Json::array({zero, zero, zero}),
                        Json::array({zero, zero, zero})});
  }

  Json conjugateComplexMatrix(Json matrix)
  {
    for (std::size_t row = 0; row < 3; ++row)
    {
      for (std::size_t column = 0; column < 3; ++column)
      {
        matrix[row][column][1] = -matrix[row][column][1].get<double>();
      }
    }
    return matrix;
  }

  void setValidVectorFitPoles(Json& input)
  {
    auto real_residue  = zeroComplexMatrix();
    real_residue[0][0] = Json::array({2.0, 0.0});
    real_residue[0][1] = Json::array({0.1, 0.0});
    real_residue[1][1] = Json::array({1.5, 0.0});
    real_residue[2][2] = Json::array({1.0, 0.0});

    auto complex_residue  = zeroComplexMatrix();
    complex_residue[0][0] = Json::array({0.5, 0.2});
    complex_residue[0][1] = Json::array({0.1, -0.1});
    complex_residue[1][1] = Json::array({0.8, 0.4});
    complex_residue[2][2] = Json::array({0.6, 0.5});

    auto& params       = findDevice(input, "vectorfit_identity")["params"];
    params["poles"]    = Json::array({Json::array({-10.0, 0.0}),
                                      Json::array({-20.0, 30.0}),
                                      Json::array({-20.0, -30.0})});
    params["residues"] = Json::array({real_residue,
                                      complex_residue,
                                      conjugateComplexMatrix(complex_residue)});
  }

  bool parserRejects(const Json& input)
  {
    std::istringstream stream(input.dump());
    return throws<>(
        [&stream]
        { static_cast<void>(EMT::parseSystemModelData(stream)); });
  }

  bool parserRejects(const std::string& input)
  {
    std::istringstream stream(input);
    return throws<>(
        [&stream]
        { static_cast<void>(EMT::parseSystemModelData(stream)); });
  }

  bool parserRejectsReplacement(const std::string& token,
                                const std::string& replacement)
  {
    auto       input    = loadFixtureText();
    const auto position = input.find(token);
    if (position == std::string::npos)
    {
      return false;
    }
    input.replace(position, token.size(), replacement);
    return parserRejects(input);
  }

  bool parserAccepts(const Json& input)
  {
    std::istringstream stream(input.dump());
    return !throws<>(
        [&stream]
        { static_cast<void>(EMT::parseSystemModelData(stream)); });
  }

  bool studyParserRejects(const Json& input)
  {
    return throws<>([&input]
                    { static_cast<void>(input.get<EMT::StudyData>()); });
  }

  template <typename Mutation>
  bool parserRejectsAfter(Mutation mutation)
  {
    auto input = loadFixtureJson();
    mutation(input);
    return parserRejects(input);
  }

  template <typename Mutation>
  bool parserAcceptsAfter(Mutation mutation)
  {
    auto input = loadFixtureJson();
    mutation(input);
    return parserAccepts(input);
  }

  /// Physical constraints are owned by `verify()`, which `allocate()` runs.
  template <typename Mutation>
  bool systemRejectsAfter(Mutation mutation)
  {
    auto input = loadFixtureJson();
    mutation(input);
    std::istringstream stream(input.dump());
    return throws<>(
        [&stream]
        {
          const auto data = EMT::parseSystemModelData(stream);
          SystemT    system(data);
          system.allocate();
        });
  }

  Json& findSubmodel(Json&              input,
                     const std::string& device_id,
                     const std::string& submodel)
  {
    return findDevice(input, device_id)["submodels"][submodel];
  }

  TestOutcome fixtureAndParser()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();

    success *= isThreeBusMutuallyCoupled(data);
    success *= data.bus[0].bus_id == 650;
    success *= data.bus[1].bus_id == 632;
    success *= data.bus[2].bus_id == 670;
    success *= data.bus[3].bus_id == 6321;
    success *= data.constant_source.size() == 4;
    success *= data.voltage_source.size() == 1;
    success *= data.line_lumped.size() == 2;
    success *= data.loadz.size() == 2;
    success *= data.switches.size() == 1;
    success *= data.vector_fit.size() == 1;
    success *= data.signal.size() == 7;
    success *= data.format_version.has_value();
    success *= data.format_version.value() == 0;
    success *= data.format_revision.has_value();
    success *= data.format_revision.value() == 1;

    success *= data.bus[0].monitored_variables.size() == 3;
    success *= data.bus[1].monitored_variables.size() == 3;
    success *= data.bus[2].monitored_variables.size() == 3;
    success *= data.voltage_source[0].monitored_variables.size() == 6;
    success *= data.line_lumped[0].monitored_variables.size() == 9;
    success *= data.line_lumped[1].monitored_variables.size() == 9;
    success *= data.loadz[0].monitored_variables.size() == 3;
    success *= data.loadz[1].monitored_variables.size() == 3;

    const std::set<EMT::BusMonitorableVariables> bus_monitors{
        EMT::BusMonitorableVariables::va,
        EMT::BusMonitorableVariables::vb,
        EMT::BusMonitorableVariables::vc};
    const std::set<EMT::LineLumpedMonitorableVariables> line_monitors{
        EMT::LineLumpedMonitorableVariables::i12a,
        EMT::LineLumpedMonitorableVariables::i12b,
        EMT::LineLumpedMonitorableVariables::i12c,
        EMT::LineLumpedMonitorableVariables::i_sh1a,
        EMT::LineLumpedMonitorableVariables::i_sh1b,
        EMT::LineLumpedMonitorableVariables::i_sh1c,
        EMT::LineLumpedMonitorableVariables::i_sh2a,
        EMT::LineLumpedMonitorableVariables::i_sh2b,
        EMT::LineLumpedMonitorableVariables::i_sh2c};
    const std::set<EMT::LoadZMonitorableVariables> load_monitors{
        EMT::LoadZMonitorableVariables::ia,
        EMT::LoadZMonitorableVariables::ib,
        EMT::LoadZMonitorableVariables::ic};
    const std::set<EMT::VoltageSourceMonitorableVariables> source_monitors{
        EMT::VoltageSourceMonitorableVariables::ea,
        EMT::VoltageSourceMonitorableVariables::eb,
        EMT::VoltageSourceMonitorableVariables::ec,
        EMT::VoltageSourceMonitorableVariables::ia,
        EMT::VoltageSourceMonitorableVariables::ib,
        EMT::VoltageSourceMonitorableVariables::ic};
    success *= data.bus[0].monitored_variables == bus_monitors;
    success *= data.bus[1].monitored_variables == bus_monitors;
    success *= data.bus[2].monitored_variables == bus_monitors;
    success *= data.line_lumped[0].monitored_variables == line_monitors;
    success *= data.line_lumped[1].monitored_variables == line_monitors;
    success *= data.loadz[0].monitored_variables == load_monitors;
    success *= data.loadz[1].monitored_variables == load_monitors;
    success *= data.voltage_source[0].monitored_variables == source_monitors;

    success *= data.bus[0].monitored_variables.contains(
        EMT::BusMonitorableVariables::va);
    success *= data.bus[0].monitored_variables.contains(
        EMT::BusMonitorableVariables::vb);
    success *= data.bus[0].monitored_variables.contains(
        EMT::BusMonitorableVariables::vc);
    success *= data.line_lumped[0].monitored_variables.contains(
        EMT::LineLumpedMonitorableVariables::i12a);
    success *= data.line_lumped[0].monitored_variables.contains(
        EMT::LineLumpedMonitorableVariables::i_sh1b);
    success *= data.line_lumped[0].monitored_variables.contains(
        EMT::LineLumpedMonitorableVariables::i_sh2c);
    success *= data.loadz[0].monitored_variables.contains(
        EMT::LoadZMonitorableVariables::ia);
    success *= data.loadz[0].monitored_variables.contains(
        EMT::LoadZMonitorableVariables::ib);
    success *= data.loadz[0].monitored_variables.contains(
        EMT::LoadZMonitorableVariables::ic);
    success *= data.voltage_source[0].monitored_variables.contains(
        EMT::VoltageSourceMonitorableVariables::ea);
    success *= data.voltage_source[0].monitored_variables.contains(
        EMT::VoltageSourceMonitorableVariables::ib);

    success *= data.line_lumped[0].buses.at(EMT::LineLumpedBuses::bus1)
               == 650;
    success *= data.line_lumped[0].buses.at(EMT::LineLumpedBuses::bus2)
               == 632;
    success *= data.loadz[0].buses.at(EMT::LoadZBuses::bus) == 670;
    success *= data.loadz[1].buses.at(EMT::LoadZBuses::bus) == 6321;
    success *= data.switches[0].buses.at(EMT::SwitchBuses::bus1) == 632;
    success *= data.switches[0].buses.at(EMT::SwitchBuses::bus2) == 6321;
    success *= data.switches[0].signal_inputs.at(
                   EMT::SwitchSignalInputs::open)
               == 0;
    success *= data.voltage_source[0].buses.at(
                   EMT::VoltageSourceBuses::bus)
               == 650;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_a)
               == 1;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_b)
               == 2;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_c)
               == 3;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_a)
               == 4;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_b)
               == 5;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_c)
               == 6;
    success *= data.constant_source[0].signal_outputs.at(
                   PhasorDynamics::ConstantSignalSourceSignalOutputs::sr)
               == 0;

    auto vector_fit_json = loadFixtureJson();
    setValidVectorFitPoles(vector_fit_json);
    std::istringstream vector_fit_stream(vector_fit_json.dump());
    const auto         vector_fit_data = EMT::parseSystemModelData(vector_fit_stream);
    const auto&        parsed_poles    = std::get<std::vector<std::complex<double>>>(
        vector_fit_data.vector_fit[0].parameters.at(
            EMT::VectorFitParameters::poles));
    const auto& parsed_residues = std::get<
        std::vector<EMT::ABCMatrix<std::complex<double>>>>(
        vector_fit_data.vector_fit[0].parameters.at(
            EMT::VectorFitParameters::residues));
    success *= parsed_poles.size() == 3;
    success *= parsed_residues.size() == 3;
    success *= parsed_poles[1] == std::complex<double>(-20.0, 30.0);
    success *= parsed_residues[1][0][0] == std::complex<double>(0.5, 0.2);
    success *= parsed_residues[2][0][0] == std::complex<double>(0.5, -0.2);

    success *= parserAcceptsAfter([](Json& input)
                                  {
      input["signals"].push_back({{"signal_id", 7},
                                   {"name", "constant_imaginary"}});
      auto& source = findDevice(input, "fault_load_open_source");
      source["params"]["Si"] = 0.25;
      source["ports"]["si"] = 7; });

    return success.report(__func__);
  }

  TestOutcome parserEnvelopeAndPortRules()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    success *= parserRejectsAfter([](Json& input)
                                  { input["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0]["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["signals"][0]["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["devices"][0]["unexpected"] = 1; });

    const std::vector<std::string> device_ids{
        "fault_load_open_source", "source_650", "line_650_632", "load_670", "fault_632_switch", "vectorfit_identity"};
    for (const auto& device_id : device_ids)
    {
      success *= parserRejectsAfter([&device_id](Json& input)
                                    { findDevice(input, device_id)["params"]["unexpected"] = 1; });
      success *= parserRejectsAfter([&device_id](Json& input)
                                    { findDevice(input, device_id)["ports"]["unexpected"] = 1; });
    }

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["class"] = "UnknownDevice"; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0]["class"] = "Bus"; });

    const std::vector<std::pair<std::string, std::vector<std::string>>>
        required_ports{
            {"line_650_632", {"bus1", "bus2"}},
            {"load_670", {"bus"}},
            {"fault_632_switch", {"bus1", "bus2", "open"}},
            {"source_650", {"bus"}},
            {"vectorfit_identity",
             {"input_a", "input_b", "input_c", "out_a", "out_b", "out_c"}},
            {"fault_load_open_source", {"sr"}}};
    for (const auto& [device_id, ports] : required_ports)
    {
      for (const auto& port : ports)
      {
        success *= parserRejectsAfter([&device_id, &port](Json& input)
                                      { findDevice(input, device_id)["ports"].erase(port); });
      }
    }

    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0].erase("number"); });
    success *= parserRejectsAfter([](Json& input)
                                  { input["signals"][0].erase("signal_id"); });
    success *= parserRejectsAfter([](Json& input)
                                  { input["devices"][0].erase("id"); });
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][1]["number"] = 650; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["signals"][1]["signal_id"] = 0; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["devices"][1]["id"] = input["devices"][0]["id"]; });

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["ports"]["bus1"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["ports"]["bus2"] = 650; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["ports"]["bus"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "fault_632_switch")["ports"]["open"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "fault_632_switch")["ports"]["bus2"] = 632; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["input_a"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["out_a"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "fault_load_open_source")["ports"]["sr"] = 999; });

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_input_a_source")["ports"]["sr"] = 0; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["out_a"] = 0; });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& ports = findDevice(input, "vectorfit_identity")["ports"];
      ports["out_b"] = ports["out_a"]; });

    success *= parserAcceptsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["omega"] = 377.0; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["format_version"] = 0.1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["format_version"] = -1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["format_version"] = 65536; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["format_revision"] = 1.5; });

    return success.report(__func__);
  }

  TestOutcome parserMatrixAndPhysicalRules()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    // Submodel coefficient blocks share one schema with the standalone
    // VectorFit device, so the same shape rules apply to every one of them.
    const std::vector<std::array<std::string, 3>> coefficient_blocks{
        {"line_650_632", "Zp", "D"},
        {"line_650_632", "Zp", "E"},
        {"line_650_632", "Yp", "D"},
        {"line_650_632", "Yp", "E"},
        {"load_670", "Z", "D"},
        {"load_670", "Z", "E"},
        {"source_650", "Z", "D"},
        {"source_650", "Z", "E"}};
    for (const auto& [device_id, submodel, parameter] : coefficient_blocks)
    {
      success *= parserRejectsAfter([&](Json& input)
                                    { findSubmodel(input, device_id, submodel).erase(parameter); });
      success *= parserRejectsAfter([&](Json& input)
                                    { findSubmodel(input, device_id, submodel)[parameter].erase(2); });
      success *= parserRejectsAfter([&](Json& input)
                                    { findSubmodel(input, device_id, submodel)[parameter][0].erase(2); });
      success *= parserRejectsAfter([&](Json& input)
                                    { findSubmodel(input, device_id, submodel)[parameter][0][0] = "not-a-number"; });
    }

    const std::vector<std::pair<std::string, std::string>> real_matrices{
        {"vectorfit_identity", "D"},
        {"vectorfit_identity", "E"}};
    for (const auto& [device_id, parameter] : real_matrices)
    {
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"].erase(parameter); });
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"][parameter].erase(2); });
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"][parameter][0].erase(2); });
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"][parameter][0][0] = "not-a-number"; });
    }

    const std::vector<std::pair<std::string, std::string>>
        other_required_parameters{
            {"line_650_632", "N"},
            {"line_650_632", "K"},
            {"line_650_632", "conductors"},
            {"line_650_632", "dx"},
            {"load_670", "N"},
            {"source_650", "N"},
            {"source_650", "E"},
            {"source_650", "phi"},
            {"source_650", "omega"},
            {"vectorfit_identity", "poles"},
            {"vectorfit_identity", "residues"}};
    for (const auto& [device_id, parameter] : other_required_parameters)
    {
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"].erase(parameter); });
    }

    // A missing or unknown submodel block is a schema error.
    const std::vector<std::pair<std::string, std::string>> required_submodels{
        {"line_650_632", "Zp"},
        {"line_650_632", "Yp"},
        {"load_670", "Z"},
        {"source_650", "Z"}};
    for (const auto& [device_id, submodel] : required_submodels)
    {
      success *= parserRejectsAfter([&device_id, &submodel](Json& input)
                                    { findDevice(input, device_id)["submodels"].erase(submodel); });
      success *= parserRejectsAfter([&device_id](Json& input)
                                    { findDevice(input, device_id)["submodels"]["Unexpected"] = Json::object(); });
    }
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["submodels"] = Json::object(); });

    // Physical constraints belong to verify(), which allocate() runs.
    for (const auto& [device_id, submodel, parameter] : coefficient_blocks)
    {
      success *= systemRejectsAfter([&](Json& input)
                                    {
        auto& matrix = findSubmodel(input, device_id, submodel)[parameter];
        matrix[0][1] = matrix[1][0].get<double>() + 1.0; });
      success *= systemRejectsAfter([&](Json& input)
                                    { findSubmodel(input, device_id, submodel)[parameter][0][0] = -1.0; });
    }

    // Rational dynamics in a submodel are not realizable yet.
    for (const auto& [device_id, submodel] : required_submodels)
    {
      success *= systemRejectsAfter([&device_id, &submodel](Json& input)
                                    {
        auto& block = findSubmodel(input, device_id, submodel);
        block["poles"]    = Json::array({Json::array({-10.0, 0.0})});
        block["residues"] = Json::array({zeroComplexMatrix()}); });
    }

    // Dimensions outside the implemented three-phase subset are rejected.
    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "load_670")["params"]["N"] = 4; });
    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["N"] = 4; });
    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["params"]["K"] = 4; });
    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["params"]["conductors"] = Json::array({1, 3, 2}); });

    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["params"]["dx"] = 0.0; });
    success *= systemRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["E"][0] = -1.0; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["E"].erase(2); });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["phi"][0] = "not-a-number"; });

    success *= parserRejectsReplacement("0.0002153051181102362", "1e309");
    success *= parserRejectsReplacement("609.6", "1e309");
    success *= parserRejectsReplacement("2401.7771198288433", "1e309");

    return success.report(__func__);
  }

  TestOutcome parserComplexMonitorAndSignalRules()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["poles"][0]
          = Json::array({-10.0}); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["poles"][0][0]
          = "not-a-number"; });
    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["residues"][0]
          = Json::array(); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["residues"][0][0]
          = Json::array(); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["residues"][0][0][0]
          = Json::array({1.0}); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      setValidVectorFitPoles(input);
      findDevice(input, "vectorfit_identity")["params"]["residues"][0][0][0][1]
          = "not-a-number"; });

    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({Json::array({-10.0, 0.0})});
      params["residues"] = Json::array(); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({Json::array({-10.0, 20.0})});
      params["residues"] = Json::array({zeroComplexMatrix()}); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({
          Json::array({-10.0, 20.0}),
          Json::array({-11.0, -20.0})});
      params["residues"] = Json::array({
          zeroComplexMatrix(), zeroComplexMatrix()}); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({
          Json::array({-10.0, 20.0}),
          Json::array({-10.0, -20.0})});
      auto residue = zeroComplexMatrix();
      auto bad_conjugate = zeroComplexMatrix();
      residue[0][0] = Json::array({1.0, 0.2});
      bad_conjugate[0][0] = Json::array({1.0, -0.1});
      params["residues"] = Json::array({residue, bad_conjugate}); });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      auto residue = zeroComplexMatrix();
      residue[0][0] = Json::array({1.0, 0.2});
      params["poles"] = Json::array({Json::array({-10.0, 0.0})});
      params["residues"] = Json::array({residue}); });
    success *= parserAcceptsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({
          Json::array({0.0, 376.99111843077515}),
          Json::array({0.0, -376.99111843077515})});
      params["residues"] = Json::array({
          zeroComplexMatrix(), zeroComplexMatrix()}); });

    const std::vector<std::string> monitored_devices{
        "line_650_632", "load_670", "source_650", "vectorfit_identity", "fault_632_switch", "fault_load_open_source"};
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0]["mon"] = Json::array({"unknown"}); });
    for (const auto& device_id : monitored_devices)
    {
      success *= parserRejectsAfter([&device_id](Json& input)
                                    { findDevice(input, device_id)["mon"] = Json::array({"unknown"}); });
    }
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0]["mon"] = "v"; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["mon"] = Json::array({1}); });
    success *= parserRejectsAfter([](Json& input)
                                  { input["monitors"] = Json::array({{{"format", "unknown"}}}); });

    return success.report(__func__);
  }

  TestOutcome studyParserRules()
  {
    TestStatus success = true;
    const auto initial_state_file =
        std::filesystem::path{EMT_IEEE13_TEST_FIXTURE}.parent_path().parent_path()
        / "ThreeBus" / "ThreeBus.initial.json";
    Json input{
        {"system_model_file", std::filesystem::path{EMT_TEST_FIXTURE}.string()},
        {"initial_state_file", initial_state_file.string()},
        {"dt_fixed", 0.000025},
        {"max_steps", 100000},
        {"suppress_algebraic_errors", true},
        {"dt_monitor", 0.0001},
        {"tmax", 0.2},
        {"output_file", "resolved.csv"},
        {"events", Json::array({{{"time", 0.1}, {"type", "signal_set"}, {"signal_id", 0}, {"value", 0.0}}, {{"time", 0.05}, {"type", "signal_set"}, {"signal_id", 0}, {"value", 1.0}}})}};

    const auto study  = input.get<EMT::StudyData>();
    success          *= study.events.size() == 2;
    success          *= study.max_steps == 100000;
    success          *= study.suppress_algebraic_errors;
    success          *= study.events[0].time == 0.05;
    success          *= study.events[1].time == 0.1;

    auto same_time              = input;
    same_time["events"]         = Json::array({{{"time", 0.05},
                                                {"type", "signal_set"},
                                                {"signal_id", 1},
                                                {"value", 0.0}},
                                               {{"time", 0.05},
                                                {"type", "signal_set"},
                                                {"signal_id", 0},
                                                {"value", 1.0}}});
    const auto same_time_study  = same_time.get<EMT::StudyData>();
    success                    *= same_time_study.events[0].signal_id == 1;
    success                    *= same_time_study.events[1].signal_id == 0;

    auto duplicate                       = same_time;
    duplicate["events"][1]["signal_id"]  = 1;
    success                             *= studyParserRejects(duplicate);

    auto final_time_event                  = input;
    final_time_event["events"][0]["time"]  = 0.2;
    success                               *= studyParserRejects(final_time_event);

    auto negative_max_steps          = input;
    negative_max_steps["max_steps"]  = -1;
    success                         *= studyParserRejects(negative_max_steps);

    auto fractional_max_steps          = input;
    fractional_max_steps["max_steps"]  = 1.5;
    success                           *= studyParserRejects(fractional_max_steps);

    auto excessive_max_steps = input;
    excessive_max_steps["max_steps"] =
        static_cast<std::uint64_t>(std::numeric_limits<long int>::max())
        + std::uint64_t{1};
    success *= studyParserRejects(excessive_max_steps);

    auto nonboolean_suppression                          = input;
    nonboolean_suppression["suppress_algebraic_errors"]  = 1;
    success                                             *= studyParserRejects(nonboolean_suppression);

    auto default_max_steps = input;
    default_max_steps.erase("max_steps");
    success *= default_max_steps.get<EMT::StudyData>().max_steps == 0;

    auto default_suppression = input;
    default_suppression.erase("suppress_algebraic_errors");
    success *= !default_suppression.get<EMT::StudyData>()
                    .suppress_algebraic_errors;

    const auto solver_file = std::filesystem::temp_directory_path()
                             / "gridkit_emt_study_parser.solver.json";
    {
      std::ofstream output(solver_file);
      output << input.dump(2);
    }
    const auto parsed           = EMT::parseStudyData(solver_file);
    const auto expected_output  = solver_file.parent_path() / "resolved.csv";
    success                    *= parsed.output_file == expected_output;
    success                    *= parsed.model_data.monitor_sink.size() == 1;
    success                    *= parsed.model_data.monitor_sink[0].file_name
               == expected_output.string();

    input["events"] = Json::array({{{"time", 0.05},
                                    {"type", "signal_set"},
                                    {"signal_id", 999},
                                    {"value", 1.0}}});
    {
      std::ofstream output(solver_file);
      output << input.dump(2);
    }
    success *= throws<>([&solver_file]
                        { static_cast<void>(EMT::parseStudyData(solver_file)); });
    std::filesystem::remove(solver_file);

    return success.report(__func__);
  }

  TestOutcome externalInitialStateContract()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();
    const auto initial_state_file =
        std::filesystem::path{EMT_IEEE13_TEST_FIXTURE}.parent_path().parent_path()
        / "ThreeBus" / "ThreeBus.initial.json";
    const auto initial_state =
        EMT::parseInitialStateData(initial_state_file, data.case_name);

    success *= initial_state.case_name == data.case_name;
    success *= initial_state.time == 0.0;
    // The switch terminal bus is algebraic and supplies no initial voltage.
    success *= initial_state.buses.size() == data.bus.size() - 1;

    auto system  = makeFixtureSystem(data);
    success     *= !throws<>([&system, &initial_state]
                         { EMT::applyInitialState(*system, initial_state); });
    for (const auto& bus_state : initial_state.buses)
    {
      const auto* values = system->getBus(bus_state.bus_id)->y().getData();
      for (std::size_t phase = 0; phase < 3; ++phase)
      {
        success *= values[phase] == bus_state.states.front().value[phase];
      }
    }
    for (const auto& component_state : initial_state.components)
    {
      const auto* component  = system->getComponent(component_state.component_id);
      const auto* layout     = dynamic_cast<const EMT::InitialStateLayout*>(component);
      success               *= layout != nullptr;
      std::vector<EMT::InitialStateVariable> variables;
      if (layout != nullptr)
      {
        layout->appendInitialStateVariables(variables);
      }
      for (const auto& state : component_state.states)
      {
        for (const auto& variable : variables)
        {
          if (variable.name != state.variable || variable.index != state.index)
          {
            continue;
          }
          for (std::size_t phase = 0; phase < 3; ++phase)
          {
            success *= component->y().getData()[variable.local_offset + phase]
                       == state.value[phase];
          }
        }
      }
    }
    for (std::size_t index = 0; index < system->size(); ++index)
    {
      success *= system->yp().getData()[index] == 0.0;
    }

    auto missing_state = initial_state;
    missing_state.buses.pop_back();
    auto missing_system  = makeFixtureSystem(data);
    success             *= throws<std::runtime_error>(
        [&missing_system, &missing_state]
        { EMT::applyInitialState(*missing_system, missing_state); });

    auto nonfinite_state = initial_state;
    nonfinite_state.buses.front().states.front().value.front() =
        std::numeric_limits<double>::infinity();
    auto nonfinite_system  = makeFixtureSystem(data);
    success               *= throws<std::runtime_error>(
        [&nonfinite_system, &nonfinite_state]
        { EMT::applyInitialState(*nonfinite_system, nonfinite_state); });

    const auto ieee_data = EMT::parseSystemModelData(
        std::filesystem::path{EMT_IEEE13_TEST_FIXTURE});
    auto ieee_state = EMT::parseInitialStateData(
        std::filesystem::path{EMT_IEEE13_TEST_FIXTURE}.parent_path()
            / "IEEE13.initial.json",
        ieee_data.case_name);
    auto ieee_system = std::make_unique<SystemT>(ieee_data);
    ieee_system->allocate();
    ieee_system->initialize();
    EMT::applyInitialState(*ieee_system, ieee_state);
    success *= ieee_system->getBus(634)->voltageClass()
               == EMT::BusVoltageClass::algebraic;
    success *= ieee_system->getBus(634)->differentiatedKCL();
    success *= ieee_system->validateInitialState() == 0;

    auto* load634 = ieee_system->getComponent(
        "load_634_referred_to_4kv_projection");
    load634->y().getData()[0] += 1.0;
    success                   *= ieee_system->validateInitialState() != 0;
    success                   *= ieee_system->getBus(634)->differentiatedKCL();
    load634->y().getData()[0] -= 1.0;
    success                   *= ieee_system->validateInitialState() == 0;

    ieee_state.buses.push_back(
        {634, {{"v", std::nullopt, {1.0, 2.0, 3.0}}}});
    success *= throws<std::runtime_error>(
        [&ieee_system, &ieee_state]
        { EMT::applyInitialState(*ieee_system, ieee_state); });

    return success.report(__func__);
  }

  TestOutcome layoutAndZeroInitialization()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    auto system  = makeFixtureSystem(data);
    success     *= system->size() == 48;
    success     *= system->getBus(650)->size() == 3;
    success     *= system->getBus(632)->size() == 3;
    success     *= system->getBus(670)->size() == 3;
    success     *= system->getBus(6321)->size() == 3;
    for (std::size_t signal_id = 0; signal_id < data.signal.size(); ++signal_id)
    {
      success *= system->getSignal(signal_id)->signalId() == signal_id;
    }

    const auto* system_y  = system->y().getData();
    success              *= system->getBus(650)->y().getData() == system_y;
    success              *= system->getBus(632)->y().getData() == system_y + 3;
    success              *= system->getBus(670)->y().getData() == system_y + 6;
    success              *= system->getBus(6321)->y().getData() == system_y + 9;
    for (std::size_t index = 0; index < system->size(); ++index)
    {
      success *= system->y().getData()[index] == 0.0;
      success *= system->yp().getData()[index] == 0.0;
    }

    const std::array<std::size_t, 11> expected_offsets{
        12, 12, 12, 12, 12, 18, 27, 36, 39, 42, 45};
    const std::array<std::size_t, 11> expected_sizes{
        0, 0, 0, 0, 6, 9, 9, 3, 3, 3, 3};
    const std::array<std::string, 11> expected_component_ids{
        "fault_load_open_source",
        "vectorfit_input_a_source",
        "vectorfit_input_b_source",
        "vectorfit_input_c_source",
        "source_650",
        "line_650_632",
        "line_632_670",
        "load_670",
        "fault_632",
        "fault_632_switch",
        "vectorfit_identity"};
    for (std::size_t id = 0; id < componentCount(data); ++id)
    {
      auto* component  = system->getComponent(id);
      success         *= component->getGridKitComponentID() == id;
      success         *= component->size() == expected_sizes[id];
      success         *= system->getComponent(expected_component_ids[id])
                 == component;
      if (component->size() == 0)
      {
        continue;
      }
      const auto offset  = component->getVariableIndex(0);
      success           *= offset == expected_offsets[id];
      success           *= component->y().getData() == system_y + offset;
      success           *= component->getResidual().getData()
                 == system->getResidual().getData() + offset;
      for (std::size_t local = 0; local < component->size(); ++local)
      {
        success *= component->getVariableIndex(local) == offset + local;
        success *= component->getResidualIndex(local) == offset + local;
      }
    }

    for (std::size_t id = 0; id < 4; ++id)
    {
      success *= dynamic_cast<ConstantSourceT*>(system->getComponent(id))
                 != nullptr;
    }
    success *= system->getSignal(0)->read() == 1.0;
    success *= system->getSignal(1)->read() == 1.0;
    success *= system->getSignal(2)->read() == -0.5;
    success *= system->getSignal(3)->read() == 0.25;
    success *= dynamic_cast<VoltageSourceT*>(system->getComponent(4))
               != nullptr;
    success *= dynamic_cast<LineLumpedT*>(system->getComponent(5)) != nullptr;
    success *= dynamic_cast<LineLumpedT*>(system->getComponent(6)) != nullptr;
    success *= dynamic_cast<LoadZT*>(system->getComponent(7)) != nullptr;
    success *= dynamic_cast<LoadZT*>(system->getComponent(8)) != nullptr;
    success *= dynamic_cast<SwitchT*>(system->getComponent(9)) != nullptr;
    success *= dynamic_cast<VectorFitT*>(system->getComponent(10)) != nullptr;
    success *= throws<std::out_of_range>([&system]
                                         { system->getComponent("unknown"); });
    success *= system->monitoring();

    // Feeder bus voltages, source and line currents, and the feeder load
    // current are differential. The switch terminal bus, the switch current
    // and the resistive fault current are algebraic.
    std::vector<bool>                                        expected_tag(system->size(), false);
    const std::array<std::pair<std::size_t, std::size_t>, 5> differential{
        {{0, 9}, {12, 15}, {18, 21}, {27, 30}, {36, 39}}};
    for (const auto& [first, last] : differential)
    {
      for (std::size_t index = first; index < last; ++index)
      {
        expected_tag[index] = true;
      }
    }
    success *= system->tag() == expected_tag;

    EMT::Bus<double, std::size_t> caller_owned_bus650(data.bus[0]);
    EMT::Bus<double, std::size_t> caller_owned_bus632(data.bus[1]);
    EMT::Bus<double, std::size_t> caller_owned_bus670(data.bus[2]);
    LineLumpedT                   caller_owned_line1(&caller_owned_bus650,
                                   &caller_owned_bus632,
                                   data.line_lumped[0]);
    LineLumpedT                   caller_owned_line2(&caller_owned_bus632,
                                   &caller_owned_bus670,
                                   data.line_lumped[1]);
    {
      SystemT caller_owned_system;
      caller_owned_system.addBus(&caller_owned_bus650);
      caller_owned_system.addBus(&caller_owned_bus632);
      caller_owned_system.addBus(&caller_owned_bus670);
      caller_owned_system.addComponent(&caller_owned_line1);
      caller_owned_system.addComponent(&caller_owned_line2);
      caller_owned_system.updateTime(0.0, 1.0);
      success *= caller_owned_system.allocate() == 0;
      success *= caller_owned_system.initialize() == 0;
    }
    success *= caller_owned_bus650.busID() == 650;
    success *= caller_owned_bus632.busID() == 632;
    success *= caller_owned_bus670.busID() == 670;

    auto       rank_deficient_data = data;
    const auto shunt               = rationalBlock(rank_deficient_data.line_lumped[0],
                                     EMT::LineLumpedSubmodels::Yp);
    setRationalBlock(rank_deficient_data.line_lumped[0],
                     EMT::LineLumpedSubmodels::Yp,
                     shunt.D,
                     EMT::ABCMatrix<double>{{{2.0e-12, -1.0e-12, -1.0e-12},
                                             {-1.0e-12, 2.0e-12, -1.0e-12},
                                             {-1.0e-12, -1.0e-12, 2.0e-12}}});
    success *= throws<>([&rank_deficient_data]
                        {
      SystemT invalid_system(rank_deficient_data);
      invalid_system.allocate(); });

    return success.report(__func__);
  }

  TestOutcome repeatedAllocationMonitor()
  {
    TestStatus success  = true;
    auto       data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    const auto output_file = std::filesystem::temp_directory_path()
                             / "gridkit_emt_repeated_allocation.csv";
    std::filesystem::remove(output_file);
    data.monitor_sink = {{Model::VariableMonitorFormat::CSV,
                          output_file.string(),
                          ","}};

    {
      SystemT system(data);
      success *= system.allocate() == 0;
      success *= system.allocate() == 0;
      system.printMonitoredVariables();
      system.stopMonitor();
    }

    std::ifstream input(output_file);
    std::string   header;
    std::string   values;
    std::string   extra;
    success *= static_cast<bool>(std::getline(input, header));
    success *= static_cast<bool>(std::getline(input, values));
    success *= !static_cast<bool>(std::getline(input, extra));
    success *= std::count(header.begin(), header.end(), ',') == 46;
    success *= std::count(values.begin(), values.end(), ',') == 46;

    const auto first_bus_voltage  = header.find("Bus_650_va");
    success                      *= first_bus_voltage != std::string::npos;
    if (first_bus_voltage != std::string::npos)
    {
      success *= header.find("Bus_650_va", first_bus_voltage + 1)
                 == std::string::npos;
    }

    input.close();
    std::filesystem::remove(output_file);
    return success.report(__func__);
  }

  TestOutcome ieee13ZeroInitializationAndClassification()
  {
    TestStatus success = true;
    const auto data    = EMT::parseSystemModelData(
        std::filesystem::path{EMT_IEEE13_TEST_FIXTURE});
    auto system  = makeFixtureSystem(data);
    success     *= data.bus.size() == 15;
    success     *= data.line_lumped.size() == 13;
    for (std::size_t index = 0; index < system->size(); ++index)
    {
      success *= system->y().getData()[index] == 0.0;
      success *= system->yp().getData()[index] == 0.0;
    }
    for (const auto& bus_data : data.bus)
    {
      auto* bus = system->getBus(bus_data.bus_id);
      // Bus 634 sits behind a transformer with no shunt admittance, and bus
      // 6711 is the terminal of the fault switch.
      if (bus_data.bus_id == 634 || bus_data.bus_id == 6711)
      {
        success *= bus->voltageClass() == EMT::BusVoltageClass::algebraic;
        success *= !bus->tag()[0] && !bus->tag()[1] && !bus->tag()[2];
      }
      else
      {
        success *= bus->voltageClass() == EMT::BusVoltageClass::differential;
        success *= bus->tag()[0] && bus->tag()[1] && bus->tag()[2];
      }
    }
    return success.report(__func__);
  }

  TestOutcome kclResetAndDirectAccumulation()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    auto* source  = dynamic_cast<VoltageSourceT*>(system->getComponent("source_650"));
    auto* line1   = dynamic_cast<LineLumpedT*>(system->getComponent("line_650_632"));
    auto* line2   = dynamic_cast<LineLumpedT*>(system->getComponent("line_632_670"));
    auto* load    = dynamic_cast<LoadZT*>(system->getComponent("load_670"));
    auto* fault   = dynamic_cast<LoadZT*>(system->getComponent("fault_632"));
    auto* device  = dynamic_cast<SwitchT*>(system->getComponent("fault_632_switch"));
    success      *= source != nullptr && line1 != nullptr && line2 != nullptr;
    success      *= load != nullptr && fault != nullptr && device != nullptr;
    if (source == nullptr || line1 == nullptr || line2 == nullptr
        || load == nullptr || fault == nullptr || device == nullptr)
    {
      return success.report(__func__);
    }

    auto* bus650  = system->getBus(650);
    auto* bus632  = system->getBus(632);
    auto* bus670  = system->getBus(670);
    auto* bus6321 = system->getBus(6321);
    bus650->Ia()  = 111.0;
    bus650->Ib()  = 222.0;
    bus650->Ic()  = 333.0;
    bus632->Ia()  = -111.0;
    bus632->Ib()  = -222.0;
    bus632->Ic()  = -333.0;
    bus670->Ia()  = 444.0;
    bus670->Ib()  = 555.0;
    bus670->Ic()  = 666.0;
    bus6321->Ia() = 777.0;
    bus6321->Ib() = 888.0;
    bus6321->Ic() = 999.0;

    device->y().getData()[0] = 13.0;
    device->y().getData()[1] = -17.0;
    device->y().getData()[2] = 19.0;
    fault->y().getData()[0]  = -13.0;
    fault->y().getData()[1]  = 17.0;
    fault->y().getData()[2]  = -19.0;
    system->y().setDataUpdated();

    success *= system->evaluateResidual() == 0;
    success *= system->getSignal(0)->read() == 1.0;

    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      const double expected650 = source->y().getData()[phase]
                                 + line1->y().getData()[3 + phase]
                                 - line1->y().getData()[phase];
      const double expected632 = line1->y().getData()[6 + phase]
                                 + line1->y().getData()[phase]
                                 + line2->y().getData()[3 + phase]
                                 - line2->y().getData()[phase]
                                 - device->y().getData()[phase];
      const double expected670 = line2->y().getData()[6 + phase]
                                 + line2->y().getData()[phase]
                                 + load->y().getData()[phase];
      const double expected6321 = device->y().getData()[phase]
                                  + fault->y().getData()[phase];
      success *= isEqual(bus650->getResidual().getData()[phase],
                         expected650,
                         1.0e-13);
      success *= isEqual(bus632->getResidual().getData()[phase],
                         expected632,
                         1.0e-13);
      success *= isEqual(bus670->getResidual().getData()[phase],
                         expected670,
                         1.0e-13);
      success *= isEqual(bus6321->getResidual().getData()[phase],
                         expected6321,
                         1.0e-13);
    }

    const std::array<double, 12> first_pass{
        bus650->Ia(), bus650->Ib(), bus650->Ic(), bus632->Ia(), bus632->Ib(), bus632->Ic(), bus670->Ia(), bus670->Ib(), bus670->Ic(), bus6321->Ia(), bus6321->Ib(), bus6321->Ic()};
    success *= system->evaluateResidual() == 0;
    const std::array<double, 12> second_pass{
        bus650->Ia(), bus650->Ib(), bus650->Ic(), bus632->Ia(), bus632->Ib(), bus632->Ic(), bus670->Ia(), bus670->Ib(), bus670->Ic(), bus6321->Ia(), bus6321->Ib(), bus6321->Ic()};
    for (std::size_t entry = 0; entry < first_pass.size(); ++entry)
    {
      success *= isEqual(second_pass[entry], first_pass[entry], 1.0e-14);
    }

    return success.report(__func__);
  }

  TestOutcome jacobianAssembly()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    system->updateTime(0.0, 17.0);
    success *= system->evaluateResidual() == 0;
    success *= system->evaluateJacobian() == 0;

    auto* jacobian  = system->getCsrJacobian();
    success        *= jacobian != nullptr;
    if (jacobian != nullptr)
    {
      success            *= jacobian->getNumRows() == system->size();
      success            *= jacobian->getNumColumns() == system->size();
      success            *= jacobian->getNnz() > system->size();
      const auto* values  = jacobian->getValues();
      for (std::size_t entry = 0; entry < jacobian->getNnz(); ++entry)
      {
        success *= std::isfinite(values[entry]);
      }
    }

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += fixtureAndParser();
  result += parserEnvelopeAndPortRules();
  result += parserMatrixAndPhysicalRules();
  result += parserComplexMonitorAndSignalRules();
  result += studyParserRules();
  result += externalInitialStateContract();
  result += layoutAndZeroInitialization();
  result += ieee13ZeroInitializationAndClassification();
  result += repeatedAllocationMonitor();
  result += kclResetAndDirectAccumulation();
  result += jacobianAssembly();
  return result.summary();
}
