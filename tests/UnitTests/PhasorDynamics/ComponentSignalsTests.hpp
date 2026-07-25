#pragma once

#include <stdexcept>

#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SignalSource/ConstantSignalSource.hpp>
#include <GridKit/Model/PhasorDynamics/SignalSource/ConstantSignalSourceData.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    enum class ComponentSignalsTestInternalVariables : size_t
    {
      output,
      MAXIMUM
    };

    enum class ComponentSignalsTestExternalVariables : size_t
    {
      input,
      MAXIMUM
    };

    template <class ScalarT, typename IdxT>
    class ComponentSignalsTests
    {
    public:
      using SignalT  = PhasorDynamics::SignalNode<ScalarT, IdxT>;
      using SignalsT = PhasorDynamics::ComponentSignals<ScalarT,
                                                        IdxT,
                                                        ComponentSignalsTestInternalVariables,
                                                        ComponentSignalsTestExternalVariables>;

      TestOutcome signalNodeDerivative()
      {
        TestStatus success = true;

        SignalT node;
        ScalarT value{2.0};
        ScalarT derivative{-3.0};
        IdxT    index{7};

        node.set(&value, &derivative, &index);
        success *= node.linked();
        success *= node.derivativeLinked();
        success *= (node.read() == value);
        success *= (node.readDerivative() == derivative);
        success *= (node.getVariableIndex() == index);

        node.set(&value, &index);
        success *= node.linked();
        success *= !node.derivativeLinked();

        bool threw = false;
        try
        {
          node.readDerivative();
        }
        catch (const std::logic_error&)
        {
          threw = true;
        }
        success *= threw;

        return success.report(__func__);
      }

      TestOutcome componentSignalsDerivative()
      {
        TestStatus success = true;

        SignalT node;
        ScalarT value{5.0};
        ScalarT derivative{6.0};
        IdxT    index{8};
        node.set(&value, &derivative, &index);

        SignalsT       signals;
        constexpr auto input = ComponentSignalsTestExternalVariables::input;
        signals.template attachSignalNode<input>(&node);

        success *= signals.template isDerivativeLinked<input>();
        success *= (signals.template readExternalVariable<input>() == value);
        success *= (signals.template readExternalVariableDerivative<input>() == derivative);

        node.set(&value, &index);
        success *= !signals.template isDerivativeLinked<input>();

        bool threw = false;
        try
        {
          signals.template readExternalVariableDerivative<input>();
        }
        catch (const std::logic_error&)
        {
          threw = true;
        }
        success *= threw;

        return success.report(__func__);
      }

      TestOutcome constantSignalSourceDerivative()
      {
        TestStatus success = true;

        using SourceT    = PhasorDynamics::ConstantSignalSource<ScalarT, IdxT>;
        using DataT      = typename SourceT::ModelDataT;
        using Parameters = typename DataT::Parameters;

        DataT data;
        data.parameters[Parameters::Sr] = 4.0;
        data.parameters[Parameters::Si] = -2.0;

        SignalT real_node;
        SignalT imag_node;
        SourceT source(data);

        constexpr auto real = PhasorDynamics::ConstantSignalSourceInternalVariables::SREAL;
        constexpr auto imag = PhasorDynamics::ConstantSignalSourceInternalVariables::SIMAG;
        source.getSignals().template assignSignalNode<real>(&real_node);
        source.getSignals().template assignSignalNode<imag>(&imag_node);

        success *= (source.allocate() == 0);
        success *= real_node.derivativeLinked();
        success *= imag_node.derivativeLinked();
        success *= (real_node.read() == ScalarT{4.0});
        success *= (imag_node.read() == ScalarT{-2.0});
        success *= (real_node.readDerivative() == ScalarT{0.0});
        success *= (imag_node.readDerivative() == ScalarT{0.0});

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit
