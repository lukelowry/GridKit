/**
 * @file SignalDelayDemo.cpp
 * @brief Standalone SystemModel example for the SignalDelay helper.
 */

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNodeData.hpp>
#include <GridKit/Model/PhasorDynamics/Source/CsvSignalSource/CsvSignalSource.hpp>
#include <GridKit/Model/PhasorDynamics/Source/SignalDelay/SignalDelay.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Solver/Dynamic/Ida.hpp>

namespace
{
  enum class DifferentialSignalSinkInternalVariables : size_t
  {
    MAXIMUM,
  };

  enum class DifferentialSignalSinkExternalVariables : size_t
  {
    INPUT,
    MAXIMUM,
  };

  template <class ScalarT, typename IdxT>
  class DifferentialSignalSink : public GridKit::PhasorDynamics::Component<ScalarT, IdxT>
  {
    using ComponentT = GridKit::PhasorDynamics::Component<ScalarT, IdxT>;

    using ComponentT::alpha_;
    using ComponentT::f_;
    using ComponentT::gridkit_component_id_;
    using ComponentT::J_;
    using ComponentT::J_cols_buffer_;
    using ComponentT::J_rows_buffer_;
    using ComponentT::J_vals_buffer_;
    using ComponentT::residual_indices_;
    using ComponentT::size_;
    using ComponentT::tag_;
    using ComponentT::variable_indices_;
    using ComponentT::y_;
    using ComponentT::yp_;

  public:
    using RealT       = typename ComponentT::RealT;
    using signal_type = GridKit::PhasorDynamics::SignalNode<ScalarT, IdxT>;

    explicit DifferentialSignalSink(signal_type* input)
    {
      signals_.template attachSignalNode<DifferentialSignalSinkExternalVariables::INPUT>(input);
      size_  = 1;
      alpha_ = 0.0;
    }

    int setGridKitComponentID(IdxT component_id) override final
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    int allocate() override final
    {
      f_.resize(1);
      y_.resize(1);
      yp_.resize(1);
      tag_.resize(1);
      variable_indices_.resize(1);
      residual_indices_.resize(1);

      this->setVariableIndex(0, 0);
      this->setResidualIndex(0, 0);
      return 0;
    }

    int verify() const override final
    {
      int ret = 0;

      if (!signals_.template isAttached<DifferentialSignalSinkExternalVariables::INPUT>())
      {
        GridKit::PhasorDynamics::Log::error() << "SignalDelayDemo: sink input signal is not attached\n";
        ret += 1;
      }
      else if (!signals_.template isLinked<DifferentialSignalSinkExternalVariables::INPUT>())
      {
        GridKit::PhasorDynamics::Log::error() << "SignalDelayDemo: sink input signal is not linked\n";
        ret += 1;
      }

      return ret;
    }

    int initialize() override final
    {
      y_[0]  = 0.0;
      yp_[0] = signals_.template readExternalVariable<DifferentialSignalSinkExternalVariables::INPUT>();
      return 0;
    }

    int tagDifferentiable() override final
    {
      tag_[0] = true;
      return 0;
    }

    int evaluateResidual() override final
    {
      f_[0] = -yp_[0] + signals_.template readExternalVariable<DifferentialSignalSinkExternalVariables::INPUT>();
      return 0;
    }

    int evaluateJacobian() override final
    {
      J_.zeroMatrix();

      if (J_rows_buffer_ == nullptr)
      {
        J_rows_buffer_ = new IdxT[2];
        J_cols_buffer_ = new IdxT[2];
        J_vals_buffer_ = new RealT[2];
      }

      J_rows_buffer_[0] = residual_indices_[0];
      J_cols_buffer_[0] = variable_indices_[0];
      J_vals_buffer_[0] = -alpha_;

      J_rows_buffer_[1] = residual_indices_[0];
      J_cols_buffer_[1] = signals_.template readExternalVariableIndex<DifferentialSignalSinkExternalVariables::INPUT>();
      J_vals_buffer_[1] = 1.0;

      J_.setValues(1.0, J_rows_buffer_, J_cols_buffer_, J_vals_buffer_, 2);
      return 0;
    }

  private:
    GridKit::PhasorDynamics::ComponentSignals<ScalarT,
                                              IdxT,
                                              DifferentialSignalSinkInternalVariables,
                                              DifferentialSignalSinkExternalVariables>
        signals_;
  };
} // namespace

int main(int argc, char** argv)
{
  using namespace GridKit::PhasorDynamics;
  using namespace GridKit::PhasorDynamics::Source;
  using namespace AnalysisManager::Sundials;

  using scalar_type = double;
  using index_type  = size_t;

  SignalNodeData<scalar_type, index_type> input_signal_data;
  input_signal_data.name      = "SignalDelay input";
  input_signal_data.signal_id = 0;

  SignalNodeData<scalar_type, index_type> delayed_signal_data;
  delayed_signal_data.name      = "SignalDelay output";
  delayed_signal_data.signal_id = 1;

  SignalNode<scalar_type, index_type> input_signal(input_signal_data);
  SignalNode<scalar_type, index_type> delayed_signal(delayed_signal_data);

  const auto example_directory = std::filesystem::path(argv[0]).parent_path();

  CsvSignalSourceData<scalar_type, index_type> source_data;
  source_data.file         = example_directory / "signaldelay_input.csv";
  source_data.time_column  = "t";
  source_data.value_column = "u";

  CsvSignalSource<scalar_type, index_type> source(&input_signal, source_data);

  SignalDelayData<scalar_type, index_type> delay_data;
  delay_data.delay         = 0.5;
  delay_data.initial_value = 0.0;
  delay_data.mode          = SignalDelayMode::LINEAR;

  SignalDelay<scalar_type, index_type> delay(&input_signal, &delayed_signal, delay_data);

  // IDA needs at least one differential equation in this standalone demo.
  // The sink observes the delayed signal without changing the recorded path.
  DifferentialSignalSink<scalar_type, index_type> sink(&delayed_signal);

  SystemModel<scalar_type, index_type> system;
  system.addSignal(&input_signal);
  system.addSignal(&delayed_signal);
  system.addComponent(&source);
  system.addComponent(&delay);
  system.addComponent(&sink);
  system.allocate();

  std::filesystem::path output_path;
  if (argc > 1)
  {
    output_path = argv[1];
  }
  else
  {
    output_path = std::filesystem::path(argv[0]).parent_path() / "signaldelay_demo.csv";
  }

  std::ofstream output(output_path);
  if (!output)
  {
    throw std::runtime_error("Unable to open output file: " + output_path.string());
  }

  output << "t,u,u_delayed\n";
  output << std::setprecision(17);

  auto record = [&](scalar_type t)
  {
    output << t << ','
           << input_signal.read() << ','
           << delayed_signal.read() << '\n';
  };

  Ida<scalar_type, index_type> ida(&system);
  ida.configureSimulation();
  ida.initializeSimulation(0.0, false);

  record(0.0);
  ida.runSimulation(10.0, 1000, record);
  output.close();

  std::cout << "Example: SignalDelayDemo\n";
  std::cout << "input: " << source_data.file << "\n";
  std::cout << "delay: " << delay_data.delay << " s\n";
  std::cout << "wrote CSV: " << output_path << "\n";

  return 0;
}
