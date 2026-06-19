#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpaceDataJSONParser.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Propagation/Propagation.hpp>
#include <GridKit/Model/EMT/Operators/Shift/Propagation/PropagationDataJSONParser.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace
{
  using scalar_type = double;
  using index_type  = size_t;
  using json        = nlohmann::json;
  namespace fs      = std::filesystem;

  using Propagation = GridKit::EMT::Operators::Shift::Propagation<scalar_type, index_type>;
  using Signal      = Propagation::SignalT;

  bool close(scalar_type value, scalar_type ref, scalar_type tol = 1.0e-12)
  {
    return std::abs(value - ref) <= tol * (1.0 + std::abs(ref));
  }

  fs::path testDir(const std::string& name)
  {
    const auto path = fs::temp_directory_path() / name;
    fs::remove_all(path);
    fs::create_directories(path / "models");
    return path;
  }

  void writeJson(const fs::path& file_path, const json& j)
  {
    std::ofstream stream(file_path);
    stream << j.dump(2) << "\n";
  }

  json complexPair(scalar_type real, scalar_type imag)
  {
    return json::array({real, imag});
  }

  json stateSpaceJson(index_type N, index_type K, bool terms = true)
  {
    json model;
    model["shape"]  = json::array({N, K});
    model["layout"] = "RowMajor";
    model["poles"]  = json::array({complexPair(-2.0, -3.0), complexPair(-2.0, 3.0)});

    json C = json::array();
    for (index_type i = 0; i < N; ++i)
    {
      const scalar_type real = 1.0 + static_cast<scalar_type>(i);
      const scalar_type imag = 0.1 * (1.0 + static_cast<scalar_type>(i));
      C.push_back(json::array({complexPair(real, imag), complexPair(real, -imag)}));
    }
    model["C"] = C;

    json B0 = json::array();
    json B1 = json::array();
    for (index_type k = 0; k < K; ++k)
    {
      const scalar_type real = 0.5 + static_cast<scalar_type>(k);
      const scalar_type imag = -0.2 * (1.0 + static_cast<scalar_type>(k));
      B0.push_back(complexPair(real, imag));
      B1.push_back(complexPair(real, -imag));
    }
    model["B"] = json::array({B0, B1});

    json d = json::array();
    json e = json::array();
    for (index_type i = 0; i < N * K; ++i)
    {
      const scalar_type scale = 1.0 + static_cast<scalar_type>(i);
      d.push_back(complexPair(terms ? 0.01 * scale : 0.0, 0.0));
      e.push_back(complexPair(terms ? 0.001 * scale : 0.0, 0.0));
    }
    model["d"] = d;
    model["e"] = e;

    model["rmse"]  = 0.0;
    model["iters"] = 1;
    return model;
  }

  json propagationJson()
  {
    return {
        {"class", "Propagation"},
        {"input", "models/Fin.model.json"},
        {"tau", "delay.json"},
        {"dt_min", 0.01},
        {"output", "models/Fout.model.json"},
    };
  }

  void writeValidModelFiles(const fs::path& dir)
  {
    writeJson(dir / "models" / "Fin.model.json", stateSpaceJson(2, 3));
    writeJson(dir / "models" / "Fout.model.json", stateSpaceJson(3, 2));
    writeJson(dir / "delay.json", json::array({0.05, 0.08}));
    writeJson(dir / "propagation.model.json", propagationJson());
  }

  GridKit::Testing::TestOutcome propagation_parser_reads_relative_model_files()
  {
    using GridKit::EMT::Operators::Shift::parsePropagationData;
    using GridKit::Testing::TestStatus;

    const auto dir = testDir("gridkit_emt_propagation_parser_valid");
    writeValidModelFiles(dir);

    const auto  data = parsePropagationData<scalar_type, index_type>(dir
                                                                    / "propagation.model.json");
    Propagation model(data, Signal{0, data.input.K, 1});

    TestStatus success  = true;
    success            *= (data.input.N == 2);
    success            *= (data.input.K == 3);
    success            *= (data.input.Q == 2);
    success            *= (data.output.N == 3);
    success            *= (data.output.K == 2);
    success            *= (data.output.Q == 2);
    success            *= (data.tau.size() == 2);
    success            *= close(data.tau[0], 0.05);
    success            *= close(data.tau[1], 0.08);
    success            *= close(data.dt_min, 0.01);
    success            *= close(data.input.D[0], 0.01);
    success            *= close(data.input.E[0], 0.001);
    success            *= (model.out().rows == data.input.K);
    success            *= (model.out().cols == 1);
    success            *= (model.size() > 0);

    fs::remove_all(dir);
    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome propagation_parser_accepts_inline_tau()
  {
    using GridKit::EMT::Operators::Shift::parsePropagationData;
    using GridKit::Testing::TestStatus;

    const auto dir = testDir("gridkit_emt_propagation_parser_inline_tau");
    writeJson(dir / "models" / "Fin.model.json", stateSpaceJson(2, 3));
    writeJson(dir / "models" / "Fout.model.json", stateSpaceJson(3, 2));
    auto model   = propagationJson();
    model["tau"] = json::array({0.04, 0.06});
    writeJson(dir / "propagation.model.json", model);

    const auto data = parsePropagationData<scalar_type, index_type>(dir
                                                                    / "propagation.model.json");

    TestStatus success  = true;
    success            *= (data.tau.size() == 2);
    success            *= close(data.tau[0], 0.04);
    success            *= close(data.tau[1], 0.06);

    fs::remove_all(dir);
    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome state_space_parser_rejects_bad_json_contracts()
  {
    using GridKit::EMT::Operators::Rational::parseStateSpaceData;
    using GridKit::Testing::TestStatus;

    TestStatus success = true;

    const auto residues_dir = testDir("gridkit_emt_state_space_parser_residues");
    auto       residues     = stateSpaceJson(2, 3);
    residues["residues"]    = json::array();
    writeJson(residues_dir / "model.json", residues);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        { parseStateSpaceData<scalar_type, index_type>(residues_dir / "model.json"); });
    fs::remove_all(residues_dir);

    const auto complex_d_dir = testDir("gridkit_emt_state_space_parser_complex_d");
    auto       complex_d     = stateSpaceJson(2, 3);
    complex_d["d"][0]        = complexPair(0.0, 0.1);
    writeJson(complex_d_dir / "model.json", complex_d);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        { parseStateSpaceData<scalar_type, index_type>(complex_d_dir / "model.json"); });
    fs::remove_all(complex_d_dir);

    const auto bad_layout_dir = testDir("gridkit_emt_state_space_parser_bad_layout");
    auto       bad_layout     = stateSpaceJson(2, 3);
    bad_layout["layout"]      = "ColumnMajor";
    writeJson(bad_layout_dir / "model.json", bad_layout);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        { parseStateSpaceData<scalar_type, index_type>(bad_layout_dir / "model.json"); });
    fs::remove_all(bad_layout_dir);

    return success.report(__func__);
  }

  GridKit::Testing::TestOutcome propagation_parser_rejects_bad_wrapper_contracts()
  {
    using GridKit::EMT::Operators::Shift::parsePropagationData;
    using GridKit::Testing::TestStatus;

    TestStatus success = true;

    const auto bad_class_dir = testDir("gridkit_emt_propagation_parser_bad_class");
    writeValidModelFiles(bad_class_dir);
    auto bad_class     = propagationJson();
    bad_class["class"] = "Overhead";
    writeJson(bad_class_dir / "propagation.model.json", bad_class);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        {
          parsePropagationData<scalar_type, index_type>(bad_class_dir
                                                        / "propagation.model.json");
        });
    fs::remove_all(bad_class_dir);

    const auto bad_tau_dir = testDir("gridkit_emt_propagation_parser_bad_tau");
    writeValidModelFiles(bad_tau_dir);
    writeJson(bad_tau_dir / "delay.json", json::array({0.05, -0.08}));
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        {
          parsePropagationData<scalar_type, index_type>(bad_tau_dir
                                                        / "propagation.model.json");
        });
    fs::remove_all(bad_tau_dir);

    const auto bad_dt_dir = testDir("gridkit_emt_propagation_parser_bad_dt");
    writeValidModelFiles(bad_dt_dir);
    auto bad_dt      = propagationJson();
    bad_dt["dt_min"] = 0.0;
    writeJson(bad_dt_dir / "propagation.model.json", bad_dt);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        {
          parsePropagationData<scalar_type, index_type>(bad_dt_dir
                                                        / "propagation.model.json");
        });
    fs::remove_all(bad_dt_dir);

    const auto bad_dims_dir = testDir("gridkit_emt_propagation_parser_bad_dims");
    writeValidModelFiles(bad_dims_dir);
    writeJson(bad_dims_dir / "models" / "Fout.model.json", stateSpaceJson(2, 2));
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        {
          parsePropagationData<scalar_type, index_type>(bad_dims_dir
                                                        / "propagation.model.json");
        });
    fs::remove_all(bad_dims_dir);

    const auto unknown_dir = testDir("gridkit_emt_propagation_parser_unknown_field");
    writeValidModelFiles(unknown_dir);
    auto unknown       = propagationJson();
    unknown["example"] = true;
    writeJson(unknown_dir / "propagation.model.json", unknown);
    success *= GridKit::Testing::throws<std::runtime_error>(
        [&]()
        {
          parsePropagationData<scalar_type, index_type>(unknown_dir
                                                        / "propagation.model.json");
        });
    fs::remove_all(unknown_dir);

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += propagation_parser_reads_relative_model_files();
  result += propagation_parser_accepts_inline_tau();
  result += state_space_parser_rejects_bad_json_contracts();
  result += propagation_parser_rejects_bad_wrapper_contracts();
  return result.summary();
}
