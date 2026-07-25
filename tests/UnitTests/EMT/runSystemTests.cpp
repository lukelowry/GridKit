#include <array>
#include <complex>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <GridKit/Testing/Testing.hpp>

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

  TestOutcome fixtureAndParser()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();

    success *= isThreeBusMutuallyCoupled(data);
    success *= data.bus[0].bus_id == 650;
    success *= data.bus[1].bus_id == 632;
    success *= data.bus[2].bus_id == 670;
    success *= data.constant_source.size() == 5;
    success *= data.voltage_source.size() == 1;
    success *= data.line_lumped.size() == 2;
    success *= data.loadz.size() == 2;
    success *= data.vector_fit.size() == 1;
    success *= data.signal.size() == 8;

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
    success *= data.loadz[0].signal_inputs.at(
                   EMT::LoadZSignalInputs::enable)
               == 0;
    success *= data.voltage_source[0].buses.at(
                   EMT::VoltageSourceBuses::bus)
               == 650;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_a)
               == 2;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_b)
               == 3;
    success *= data.vector_fit[0].signal_inputs.at(
                   EMT::VectorFitSignalInputs::input_c)
               == 4;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_a)
               == 5;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_b)
               == 6;
    success *= data.vector_fit[0].signal_outputs.at(
                   EMT::VectorFitSignalOutputs::out_c)
               == 7;
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
      input["signals"].push_back({{"signal_id", 8},
                                   {"name", "constant_imaginary"}});
      auto& source = findDevice(input, "normal_load_enable_source");
      source["params"]["Si"] = 0.25;
      source["ports"]["si"] = 8; });

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
                                  { input["buses"][0]["init"]["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["signals"][0]["unexpected"] = 1; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["devices"][0]["unexpected"] = 1; });

    const std::vector<std::string> device_ids{
        "normal_load_enable_source", "source_650", "line_650_632", "load_670", "vectorfit_identity"};
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
            {"load_670", {"bus", "enable"}},
            {"source_650", {"bus"}},
            {"vectorfit_identity",
             {"input_a", "input_b", "input_c", "out_a", "out_b", "out_c"}},
            {"normal_load_enable_source", {"sr"}}};
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
                                  { findDevice(input, "load_670")["ports"]["enable"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["input_a"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["out_a"] = 999; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "normal_load_enable_source")["ports"]["sr"] = 999; });

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "fault_load_enable_source")["ports"]["sr"] = 0; });
    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "vectorfit_identity")["ports"]["out_a"] = 0; });
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& ports = findDevice(input, "vectorfit_identity")["ports"];
      ports["out_b"] = ports["out_a"]; });

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "source_650")["params"]["omega"] = 377.0; });
    success *= parserRejectsAfter([](Json& input)
                                  { input["header"]["omega0"] = -1.0; });

    return success.report(__func__);
  }

  TestOutcome parserMatrixAndPhysicalRules()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    const std::vector<std::pair<std::string, std::string>> real_matrices{
        {"line_650_632", "Rp"},
        {"line_650_632", "Lp"},
        {"line_650_632", "Gp"},
        {"line_650_632", "Cp"},
        {"load_670", "R"},
        {"load_670", "L"},
        {"source_650", "Rs"},
        {"source_650", "Ls"},
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
            {"line_650_632", "dx"},
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

    const std::vector<std::pair<std::string, std::string>>
        constrained_matrices{
            {"line_650_632", "Rp"},
            {"line_650_632", "Lp"},
            {"line_650_632", "Gp"},
            {"line_650_632", "Cp"},
            {"load_670", "R"},
            {"load_670", "L"},
            {"source_650", "Rs"},
            {"source_650", "Ls"}};
    for (const auto& [device_id, parameter] : constrained_matrices)
    {
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    {
        auto& matrix = findDevice(input, device_id)["params"][parameter];
        matrix[0][1] = matrix[1][0].get<double>() + 1.0; });
      success *= parserRejectsAfter([&device_id, &parameter](Json& input)
                                    { findDevice(input, device_id)["params"][parameter][0][0] = -1.0; });
    }

    success *= parserRejectsAfter([](Json& input)
                                  { findDevice(input, "line_650_632")["params"]["dx"] = 0.0; });
    success *= parserRejectsAfter([](Json& input)
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
                                  { input["buses"][0]["init"]["va"] = Json::array({1.0}); });
    success *= parserRejectsAfter([](Json& input)
                                  { input["buses"][0]["init"]["va"][1] = "not-a-number"; });
    success *= parserRejectsReplacement("2401.357455418130", "1e309");

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
    success *= parserRejectsAfter([](Json& input)
                                  {
      auto& params = findDevice(input, "vectorfit_identity")["params"];
      params["poles"] = Json::array({
          Json::array({0.0, 376.99111843077515}),
          Json::array({0.0, -376.99111843077515})});
      params["residues"] = Json::array({
          zeroComplexMatrix(), zeroComplexMatrix()}); });

    const std::vector<std::string> monitored_devices{
        "line_650_632", "load_670", "source_650", "vectorfit_identity", "normal_load_enable_source"};
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

  TestOutcome layoutAndInitialization()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);

    auto system  = makeFixtureSystem(data);
    success     *= system->size() == 42;
    success     *= system->getBus(650)->size() == 3;
    success     *= system->getBus(632)->size() == 3;
    success     *= system->getBus(670)->size() == 3;
    for (std::size_t signal_id = 0; signal_id < data.signal.size(); ++signal_id)
    {
      success *= system->getSignal(signal_id)->signalId() == signal_id;
    }

    const auto* system_y  = system->y().getData();
    success              *= system->getBus(650)->y().getData() == system_y;
    success              *= system->getBus(632)->y().getData() == system_y + 3;
    success              *= system->getBus(670)->y().getData() == system_y + 6;

    const std::array<std::size_t, 11> expected_offsets{
        9, 9, 9, 9, 9, 9, 15, 24, 33, 36, 39};
    const std::array<std::size_t, 11> expected_sizes{
        0, 0, 0, 0, 0, 6, 9, 9, 3, 3, 3};
    for (std::size_t id = 0; id < componentCount(data); ++id)
    {
      auto* component  = system->getComponent(id);
      success         *= component->getGridKitComponentID() == id;
      success         *= component->size() == expected_sizes[id];
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

    for (std::size_t id = 0; id < 5; ++id)
    {
      success *= dynamic_cast<ConstantSourceT*>(system->getComponent(id))
                 != nullptr;
    }
    success *= system->getSignal(0)->read() == 1.0;
    success *= system->getSignal(1)->read() == 0.0;
    success *= system->getSignal(2)->read() == 1.0;
    success *= system->getSignal(3)->read() == -0.5;
    success *= system->getSignal(4)->read() == 0.25;
    success *= dynamic_cast<VoltageSourceT*>(system->getComponent(5))
               != nullptr;
    success *= dynamic_cast<LineLumpedT*>(system->getComponent(6)) != nullptr;
    success *= dynamic_cast<LineLumpedT*>(system->getComponent(7)) != nullptr;
    success *= dynamic_cast<LoadZT*>(system->getComponent(8)) != nullptr;
    success *= dynamic_cast<LoadZT*>(system->getComponent(9)) != nullptr;
    success *= dynamic_cast<VectorFitT*>(system->getComponent(10)) != nullptr;

    success *= normalizedResidualInfinityNorm(*system) < 1.0e-9;
    success *= system->monitoring();

    std::vector<bool> expected_tag(system->size(), false);
    for (std::size_t index = 0; index < 12; ++index)
    {
      expected_tag[index] = true;
    }
    for (std::size_t index = 15; index < 18; ++index)
    {
      expected_tag[index] = true;
    }
    for (std::size_t index = 24; index < 27; ++index)
    {
      expected_tag[index] = true;
    }
    for (std::size_t index = 33; index < 39; ++index)
    {
      expected_tag[index] = true;
    }
    success *= system->tag() == expected_tag;

    EMT::Bus<double, std::size_t> caller_owned_bus(data.bus[0], data.omega0);
    {
      SystemT caller_owned_system;
      caller_owned_system.addBus(&caller_owned_bus);
      caller_owned_system.updateTime(0.0, 1.0);
      success *= caller_owned_system.allocate() == 0;
      success *= caller_owned_system.initialize() == 0;
    }
    success *= caller_owned_bus.busID() == 650;

    return success.report(__func__);
  }

  TestOutcome kclResetAndDirectAccumulation()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    auto* source  = dynamic_cast<VoltageSourceT*>(system->getComponent(5));
    auto* line1   = dynamic_cast<LineLumpedT*>(system->getComponent(6));
    auto* line2   = dynamic_cast<LineLumpedT*>(system->getComponent(7));
    auto* load    = dynamic_cast<LoadZT*>(system->getComponent(8));
    auto* fault   = dynamic_cast<LoadZT*>(system->getComponent(9));
    success      *= source != nullptr && line1 != nullptr && line2 != nullptr;
    success      *= load != nullptr && fault != nullptr;
    if (source == nullptr || line1 == nullptr || line2 == nullptr
        || load == nullptr || fault == nullptr)
    {
      return success.report(__func__);
    }

    auto* bus650 = system->getBus(650);
    auto* bus632 = system->getBus(632);
    auto* bus670 = system->getBus(670);
    bus650->Ia() = 111.0;
    bus650->Ib() = 222.0;
    bus650->Ic() = 333.0;
    bus632->Ia() = -111.0;
    bus632->Ib() = -222.0;
    bus632->Ic() = -333.0;
    bus670->Ia() = 444.0;
    bus670->Ib() = 555.0;
    bus670->Ic() = 666.0;

    success                    *= system->evaluateResidual() == 0;
    const double normal_enable  = system->getSignal(0)->read();
    const double fault_enable   = system->getSignal(1)->read();
    success                    *= normal_enable == 1.0;
    success                    *= fault_enable == 0.0;

    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      const double expected650 = source->y().getData()[phase]
                                 + line1->y().getData()[3 + phase]
                                 - line1->y().getData()[phase];
      const double expected632 = line1->y().getData()[6 + phase]
                                 + line1->y().getData()[phase]
                                 + line2->y().getData()[3 + phase]
                                 - line2->y().getData()[phase]
                                 + fault_enable * fault->y().getData()[phase];
      const double expected670 = line2->y().getData()[6 + phase]
                                 + line2->y().getData()[phase]
                                 + normal_enable * load->y().getData()[phase];
      success *= isEqual(bus650->getResidual().getData()[phase],
                         expected650,
                         1.0e-13);
      success *= isEqual(bus632->getResidual().getData()[phase],
                         expected632,
                         1.0e-13);
      success *= isEqual(bus670->getResidual().getData()[phase],
                         expected670,
                         1.0e-13);
    }

    const std::array<double, 9> first_pass{
        bus650->Ia(), bus650->Ib(), bus650->Ic(), bus632->Ia(), bus632->Ib(), bus632->Ic(), bus670->Ia(), bus670->Ib(), bus670->Ic()};
    success *= system->evaluateResidual() == 0;
    const std::array<double, 9> second_pass{
        bus650->Ia(), bus650->Ib(), bus650->Ic(), bus632->Ia(), bus632->Ib(), bus632->Ic(), bus670->Ia(), bus670->Ib(), bus670->Ic()};
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
  result += layoutAndInitialization();
  result += kclResetAndDirectAccumulation();
  result += jacobianAssembly();
  return result.summary();
}
