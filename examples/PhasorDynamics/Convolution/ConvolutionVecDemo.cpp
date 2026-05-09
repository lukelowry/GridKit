/**
 * @file ConvolutionVecDemo.cpp
 * @brief Standalone SystemModel example for the ConvolutionVec helper.
 */

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVec/ConvolutionVec.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNodeData.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSource.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

namespace
{
  template <typename ScalarT, typename IdxT>
  auto makeSignalData(const std::string& name, IdxT id)
      -> GridKit::PhasorDynamics::SignalNodeData<ScalarT, IdxT>
  {
    GridKit::PhasorDynamics::SignalNodeData<ScalarT, IdxT> data;
    data.name      = name;
    data.signal_id = id;
    return data;
  }
} // namespace

int main(int argc, char** argv)
{
  using namespace GridKit::PhasorDynamics;
  using namespace GridKit::PhasorDynamics::Convolution;
  using namespace GridKit::PhasorDynamics::Source;
  using namespace AnalysisManager::Sundials;

  using scalar_type = double;
  using index_type  = size_t;

  constexpr scalar_type pi                = 3.141592653589793238462643383279502884;
  constexpr scalar_type sqrt3             = 1.732050807568877293527446341505872367;
  constexpr scalar_type angular_frequency = 2.0 * pi;
  constexpr scalar_type modal_gain        = 8.0;
  constexpr scalar_type shaping_gain      = 2.5;

  SignalNode<scalar_type, index_type> input_a(makeSignalData<scalar_type, index_type>("phase A input", 0));
  SignalNode<scalar_type, index_type> input_b(makeSignalData<scalar_type, index_type>("phase B input", 1));
  SignalNode<scalar_type, index_type> input_c(makeSignalData<scalar_type, index_type>("phase C input", 2));
  SignalNode<scalar_type, index_type> output_a(makeSignalData<scalar_type, index_type>("phase A output", 3));
  SignalNode<scalar_type, index_type> output_b(makeSignalData<scalar_type, index_type>("phase B output", 4));
  SignalNode<scalar_type, index_type> output_c(makeSignalData<scalar_type, index_type>("phase C output", 5));

  const auto example_directory = std::filesystem::path(argv[0]).parent_path();
  const auto input_path        = example_directory / "convolutionvec_three_phase_input.csv";

  auto makeSourceData = [&](const std::string& column)
  {
    CsvSignalSourceData<scalar_type, index_type> data;
    data.file         = input_path;
    data.time_column  = "t";
    data.value_column = column;
    return data;
  };

  CsvSignalSource<scalar_type, index_type> source_a(&input_a, makeSourceData("ua"));
  CsvSignalSource<scalar_type, index_type> source_b(&input_b, makeSourceData("ub"));
  CsvSignalSource<scalar_type, index_type> source_c(&input_c, makeSourceData("uc"));

  ConvolutionVecData<scalar_type, index_type> convolution_data;
  convolution_data.dimension = 3;
  convolution_data.d         = {0.10, 0.00, 0.00, 0.00, 0.10, 0.00, 0.00, 0.00, 0.10};
  convolution_data.e         = std::vector<scalar_type>(9, 0.0);
  convolution_data.p         = {-angular_frequency, -angular_frequency, -3.0 * angular_frequency};
  convolution_data.b         = {1.0, -0.5, -0.5, 0.0, 0.8660254037844386, -0.8660254037844386, 1.0, -0.5, -0.5};
  convolution_data.c         = {0.0, 0.5 * sqrt3 * modal_gain, -0.5 * sqrt3 * modal_gain, -modal_gain, 0.5 * modal_gain, 0.5 * modal_gain, shaping_gain, -0.5 * shaping_gain, -0.5 * shaping_gain};
  convolution_data.u0        = {1.0, -0.5, -0.5};
  convolution_data.up0       = {0.0,
                                0.5 * sqrt3 * angular_frequency,
                                -0.5 * sqrt3 * angular_frequency};

  std::vector<SignalNode<scalar_type, index_type>*> input_signals{&input_a, &input_b, &input_c};
  std::vector<SignalNode<scalar_type, index_type>*> output_signals{&output_a, &output_b, &output_c};

  ConvolutionVec<scalar_type, index_type> convolution(input_signals, output_signals, convolution_data);

  SystemModel<scalar_type, index_type> system;
  system.addSignal(&input_a);
  system.addSignal(&input_b);
  system.addSignal(&input_c);
  system.addSignal(&output_a);
  system.addSignal(&output_b);
  system.addSignal(&output_c);
  system.addComponent(&source_a);
  system.addComponent(&source_b);
  system.addComponent(&source_c);
  system.addComponent(&convolution);
  system.allocate();

  std::filesystem::path output_path;
  if (argc > 1)
  {
    output_path = argv[1];
  }
  else
  {
    output_path = example_directory / "convolutionvec_demo.csv";
  }

  std::ofstream output(output_path);
  if (!output)
  {
    throw std::runtime_error("Unable to open output file: " + output_path.string());
  }

  output << "t,ua,ub,uc,za,zb,zc\n";
  output << std::setprecision(17);

  auto record = [&](scalar_type t)
  {
    output << t << ','
           << input_a.read() << ','
           << input_b.read() << ','
           << input_c.read() << ','
           << output_a.read() << ','
           << output_b.read() << ','
           << output_c.read() << '\n';
  };

  Ida<scalar_type, index_type> ida(&system);
  ida.configureSimulation();
  ida.initializeSimulation(0.0, false);

  record(0.0);
  ida.runSimulation(2.0, 200, record);
  output.close();

  std::cout << "Example: ConvolutionVecDemo\n";
  std::cout << "input: " << input_path << "\n";
  std::cout << "output: z(t) from three-phase ConvolutionVec\n";
  std::cout << "wrote CSV: " << output_path << "\n";

  return 0;
}
