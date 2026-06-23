#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    enum class ComponentSignalsTestInternalVariables : size_t
    {
      V,
      MAXIMUM,
    };

    enum class ComponentSignalsTestExternalVariables : size_t
    {
      I,
      MAXIMUM,
    };

    template <typename scalar_type, typename index_type>
    class ComponentSignalsVectorModel : public PhasorDynamics::Component<scalar_type, index_type>
    {
      using PhasorDynamics::Component<scalar_type, index_type>::size_;
      using PhasorDynamics::Component<scalar_type, index_type>::y_;
      using PhasorDynamics::Component<scalar_type, index_type>::yp_;
      using PhasorDynamics::Component<scalar_type, index_type>::f_;
      using PhasorDynamics::Component<scalar_type, index_type>::tag_;
      using PhasorDynamics::Component<scalar_type, index_type>::abs_tol_;
      using PhasorDynamics::Component<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::Component<scalar_type, index_type>::residual_indices_;

    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using RealT   = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using SignalT = PhasorDynamics::SignalNode<ScalarT, IdxT>;
      using SignalsT =
          PhasorDynamics::ComponentSignals<ScalarT,
                                           IdxT,
                                           ComponentSignalsTestInternalVariables,
                                           ComponentSignalsTestExternalVariables>;

      explicit ComponentSignalsVectorModel(std::size_t N)
        : N_(N),
          signals_(N)
      {
        size_ = static_cast<IdxT>(N_);
      }

      auto getSignals() -> SignalsT&
      {
        return signals_;
      }

      int setGridKitComponentID(IdxT) override final
      {
        return 0;
      }

      int verify() const override final
      {
        return 0;
      }

      int allocate() override final
      {
        y_.resize(N_);
        yp_.resize(N_);
        f_.resize(N_);
        tag_.resize(N_);
        abs_tol_.resize(N_);
        variable_indices_.resize(N_);
        residual_indices_.resize(N_);
        ws_.resize(N_);
        ws_indices_.resize(N_);

        for (std::size_t n = 0; n < N_; ++n)
        {
          this->setVariableIndex(static_cast<IdxT>(n), static_cast<IdxT>(n));
          this->setResidualIndex(static_cast<IdxT>(n), static_cast<IdxT>(n));

          if (signals_.template isAssigned<ComponentSignalsTestInternalVariables::V>(n))
          {
            signals_.template getSignalNode<ComponentSignalsTestInternalVariables::V>(n)->set(
                &y_[n],
                &yp_[n],
                &variable_indices_[n],
                true);
          }
        }

        return 0;
      }

      int initialize() override final
      {
        std::fill(y_.begin(), y_.end(), ScalarT{0.0});
        std::fill(yp_.begin(), yp_.end(), ScalarT{0.0});
        return 0;
      }

      int tagDifferentiable() override final
      {
        std::fill(tag_.begin(), tag_.end(), true);
        return 0;
      }

      int setAbsoluteTolerance(RealT rel_tol) override final
      {
        std::fill(abs_tol_.begin(), abs_tol_.end(), rel_tol);
        return 0;
      }

      int evaluateResidual() override final
      {
        signals_.template readExternalVariables<ComponentSignalsTestExternalVariables::I>(ws_.data());
        signals_.template readExternalVariableIndices<ComponentSignalsTestExternalVariables::I>(ws_indices_.data());

        for (std::size_t n = 0; n < N_; ++n)
        {
          f_[n] = ws_[n] - y_[n];
        }

        return 0;
      }

      int evaluateJacobian() override final
      {
        return 0;
      }

      void setLocal(std::size_t n, ScalarT y, ScalarT yp)
      {
        y_[n]  = y;
        yp_[n] = yp;
      }

      const std::vector<IdxT>& inputIndices() const
      {
        return ws_indices_;
      }

    private:
      std::size_t N_{0};
      SignalsT    signals_;

      std::vector<ScalarT> ws_;
      std::vector<IdxT>    ws_indices_;
    };

    template <typename scalar_type, typename index_type>
    class ComponentSignalsTests
    {
    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using SignalT = PhasorDynamics::SignalNode<ScalarT, IdxT>;
      using SignalsT =
          PhasorDynamics::ComponentSignals<ScalarT,
                                           IdxT,
                                           ComponentSignalsTestInternalVariables,
                                           ComponentSignalsTestExternalVariables>;

      TestOutcome scalarCompatibility()
      {
        TestStatus success = true;

        SignalsT signals;
        SignalT  output;
        SignalT  input;
        ScalarT  y{2.0};
        ScalarT  yp{3.0};
        ScalarT  i{4.0};
        ScalarT  ip{5.0};
        IdxT     y_index{7};
        IdxT     i_index{8};

        output.set(&y, &yp, &y_index, true);
        input.set(&i, &ip, &i_index, true);

        signals.template assignSignalNode<ComponentSignalsTestInternalVariables::V>(&output);
        signals.template attachSignalNode<ComponentSignalsTestExternalVariables::I>(&input);

        success *= signals.portSize() == 1;
        success *= signals.template isAssigned<ComponentSignalsTestInternalVariables::V>();
        success *= signals.template isAttached<ComponentSignalsTestExternalVariables::I>();
        success *= signals.template isLinked<ComponentSignalsTestExternalVariables::I>();
        success *= signals.template getSignalNode<ComponentSignalsTestInternalVariables::V>() == &output;
        success *= isEqual(signals.template readExternalVariable<ComponentSignalsTestExternalVariables::I>(), i);
        success *= isEqual(signals.template readExternalVariableDerivative<ComponentSignalsTestExternalVariables::I>(), ip);
        success *= signals.template readExternalVariableIndex<ComponentSignalsTestExternalVariables::I>() == i_index;

        signals.template writeExternalVariable<ComponentSignalsTestExternalVariables::I>(ScalarT{9.0});
        success *= isEqual(i, ScalarT{9.0});

        return success.report(__func__);
      }

      TestOutcome vectorSizeOne()
      {
        return vectorPort(1).report(__func__);
      }

      TestOutcome vectorSizeThree()
      {
        return vectorPort(3).report(__func__);
      }

      TestOutcome vectorSizeFive()
      {
        return vectorPort(5).report(__func__);
      }

      TestOutcome invalidPorts()
      {
        TestStatus success = true;

        SignalT               node;
        SignalsT              signals(3);
        std::vector<SignalT*> wrong_width{&node, &node};

        success *= throws<std::logic_error>(
            [&]()
            { SignalsT bad_signals(0); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(wrong_width); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(wrong_width); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template attachSignalNode<ComponentSignalsTestExternalVariables::I>(3, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template assignSignalNode<ComponentSignalsTestInternalVariables::V>(3, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template readExternalVariable<ComponentSignalsTestExternalVariables::I>(0); });

        return success.report(__func__);
      }

      TestOutcome componentUsage()
      {
        TestStatus success = true;

        constexpr std::size_t                      N = 4;
        ComponentSignalsVectorModel<ScalarT, IdxT> model(N);

        std::vector<SignalT>  v_nodes(N);
        std::vector<SignalT>  i_nodes(N);
        std::vector<SignalT*> v_node_ptrs(N);
        std::vector<SignalT*> i_node_ptrs(N);
        std::vector<ScalarT>  currents{10.0, 20.0, 30.0, 40.0};
        std::vector<ScalarT>  current_derivatives{1.0, 2.0, 3.0, 4.0};
        std::vector<IdxT>     current_indices{10, 11, 12, 13};

        for (std::size_t n = 0; n < N; ++n)
        {
          v_node_ptrs[n] = &v_nodes[n];
          i_node_ptrs[n] = &i_nodes[n];
          i_nodes[n].set(&currents[n], &current_derivatives[n], &current_indices[n], true);
        }

        model.getSignals().template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(v_node_ptrs);
        model.getSignals().template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(i_node_ptrs);
        model.allocate();

        for (std::size_t n = 0; n < N; ++n)
        {
          model.setLocal(n, ScalarT{100.0 + static_cast<ScalarT>(n)}, ScalarT{200.0 + static_cast<ScalarT>(n)});
        }

        model.evaluateResidual();

        const auto& residual = model.getResidual();
        const auto& indices  = model.inputIndices();

        for (std::size_t n = 0; n < N; ++n)
        {
          success *= isEqual(v_nodes[n].read(), ScalarT{100.0 + static_cast<ScalarT>(n)});
          success *= isEqual(v_nodes[n].readDerivative(), ScalarT{200.0 + static_cast<ScalarT>(n)});
          success *= v_nodes[n].getVariableIndex() == static_cast<IdxT>(n);
          success *= isEqual(residual[n], currents[n] - ScalarT{100.0 + static_cast<ScalarT>(n)});
          success *= indices[n] == current_indices[n];
        }

        return success.report(__func__);
      }

    private:
      TestStatus vectorPort(std::size_t N)
      {
        TestStatus success = true;

        SignalsT              signals(N);
        std::vector<SignalT>  input_nodes(N);
        std::vector<SignalT>  output_nodes(N);
        std::vector<SignalT*> input_node_ptrs(N);
        std::vector<SignalT*> output_node_ptrs(N);
        std::vector<ScalarT>  values(N);
        std::vector<ScalarT>  derivatives(N);
        std::vector<IdxT>     indices(N);

        for (std::size_t n = 0; n < N; ++n)
        {
          values[n]           = ScalarT{10.0 + static_cast<ScalarT>(n)};
          derivatives[n]      = ScalarT{20.0 + static_cast<ScalarT>(n)};
          indices[n]          = static_cast<IdxT>(30 + n);
          input_node_ptrs[n]  = &input_nodes[n];
          output_node_ptrs[n] = &output_nodes[n];
          input_nodes[n].set(&values[n], &derivatives[n], &indices[n], true);
        }

        signals.template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(input_node_ptrs);
        signals.template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(output_node_ptrs);

        std::vector<ScalarT> read_values(N);
        std::vector<ScalarT> read_derivatives(N);
        std::vector<IdxT>    read_indices(N);

        signals.template readExternalVariables<ComponentSignalsTestExternalVariables::I>(read_values.data());
        signals.template readExternalVariableDerivatives<ComponentSignalsTestExternalVariables::I>(read_derivatives.data());
        signals.template readExternalVariableIndices<ComponentSignalsTestExternalVariables::I>(read_indices.data());

        success *= signals.portSize() == N;
        for (std::size_t n = 0; n < N; ++n)
        {
          success *= signals.template isAttached<ComponentSignalsTestExternalVariables::I>(n);
          success *= signals.template isAssigned<ComponentSignalsTestInternalVariables::V>(n);
          success *= signals.template isLinked<ComponentSignalsTestExternalVariables::I>(n);
          success *= signals.template isExternalVariableDifferential<ComponentSignalsTestExternalVariables::I>(n);
          success *= signals.template getSignalNode<ComponentSignalsTestInternalVariables::V>(n) == &output_nodes[n];
          success *= isEqual(read_values[n], values[n]);
          success *= isEqual(read_derivatives[n], derivatives[n]);
          success *= read_indices[n] == indices[n];
        }

        return success;
      }
    };
  } // namespace Testing
} // namespace GridKit
