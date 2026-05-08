/**
 * @file ConvolutionVFDemo.cpp
 * @brief Standalone SystemModel example for the ConvolutionVF helper.
 */

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Convolution/ConvolutionVF/ConvolutionVF.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNodeData.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSource.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

int main(int argc, char** argv)
{
  using namespace GridKit::PhasorDynamics;
  using namespace GridKit::PhasorDynamics::Convolution;
  using namespace GridKit::PhasorDynamics::Source;
  using namespace AnalysisManager::Sundials;

  using scalar_type = double;
  using index_type  = size_t;

  constexpr scalar_type d   = 0.2;
  constexpr scalar_type e   = 0.05;
  constexpr scalar_type u0  = 1.0;
  constexpr scalar_type up0 = 0.0;

  const std::vector<scalar_type> poles{-2.0, -7.0};
  const std::vector<scalar_type> residues{1.5, -0.4};

  SignalNodeData<scalar_type, index_type> input_signal_data;
  input_signal_data.name      = "ConvolutionVF input";
  input_signal_data.signal_id = 0;

  SignalNodeData<scalar_type, index_type> output_signal_data;
  output_signal_data.name      = "ConvolutionVF output";
  output_signal_data.signal_id = 1;

  SignalNode<scalar_type, index_type> input_signal(input_signal_data);
  SignalNode<scalar_type, index_type> output_signal(output_signal_data);

  const auto example_directory = std::filesystem::path(argv[0]).parent_path();

  CsvSignalSourceData<scalar_type, index_type> source_data;
  source_data.file         = example_directory / "convolutionvf_input.csv";
  source_data.time_column  = "t";
  source_data.value_column = "u";

  CsvSignalSource<scalar_type, index_type> source(&input_signal, source_data);

  ConvolutionVFData<scalar_type, index_type> data;
  data.d   = d;
  data.e   = e;
  data.u0  = u0;
  data.up0 = up0;
  data.p   = poles;
  data.r   = residues;

  ConvolutionVF<scalar_type, index_type> convolution(&input_signal, &output_signal, data);

  SystemModel<scalar_type, index_type> system;
  system.addSignal(&input_signal);
  system.addSignal(&output_signal);
  system.addComponent(&source);
  system.addComponent(&convolution);
  system.allocate();

  std::filesystem::path output_path;
  if (argc > 1)
  {
    output_path = argv[1];
  }
  else
  {
    output_path = std::filesystem::path(argv[0]).parent_path() / "convolutionvf_demo.csv";
  }

  std::ofstream output(output_path);
  if (!output)
  {
    throw std::runtime_error("Unable to open output file: " + output_path.string());
  }

  output << "t,u,z\n";
  output << std::setprecision(17);

  auto record = [&](scalar_type t)
  {
    output << t << ','
           << input_signal.read() << ','
           << output_signal.read() << '\n';
  };

  Ida<scalar_type, index_type> ida(&system);
  ida.configureSimulation();
  ida.initializeSimulation(0.0, false);

  record(0.0);
  ida.runSimulation(10.0, 1000, record);
  output.close();

  std::cout << "Example: ConvolutionVFDemo\n";
  std::cout << "input: " << source_data.file << "\n";
  std::cout << "output: z(t) from ConvolutionVF\n";
  std::cout << "wrote CSV: " << output_path << "\n";

  return 0;
}
