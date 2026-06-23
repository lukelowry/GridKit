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
    class ComponentSignalsRepeatedPortModel : public PhasorDynamics::Component<scalar_type, index_type>
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

      ComponentSignalsRepeatedPortModel(std::size_t N, std::size_t input_count)
        : N_(N),
          input_count_(input_count),
          signals_(N)
      {
        signals_.template setExternalPortCount<ComponentSignalsTestExternalVariables::I>(
            input_count_);
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
        ws_.resize(input_count_ * N_);
        ws_indices_.resize(input_count_ * N_);

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
        for (std::size_t port = 0; port < input_count_; ++port)
        {
          const std::size_t offset = port * N_;
          signals_.template readExternalVariables<ComponentSignalsTestExternalVariables::I>(
              port,
              ws_.data() + offset);
          signals_.template readExternalVariableIndices<ComponentSignalsTestExternalVariables::I>(
              port,
              ws_indices_.data() + offset);
        }

        for (std::size_t n = 0; n < N_; ++n)
        {
          f_[n] = -y_[n];
          for (std::size_t port = 0; port < input_count_; ++port)
          {
            f_[n] += ws_[port * N_ + n];
          }
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

      const std::vector<ScalarT>& inputValues() const
      {
        return ws_;
      }

      const std::vector<IdxT>& inputIndices() const
      {
        return ws_indices_;
      }

    private:
      std::size_t N_{0};
      std::size_t input_count_{0};
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

      TestOutcome repeatedExternalPorts()
      {
        TestStatus success = true;

        constexpr std::size_t N          = 3;
        constexpr std::size_t port_count = 2;

        SignalsT signals(N);
        signals.template setExternalPortCount<ComponentSignalsTestExternalVariables::I>(port_count);

        std::vector<SignalT>               input_nodes(port_count * N);
        std::vector<std::vector<SignalT*>> input_node_ptrs(port_count, std::vector<SignalT*>(N));
        std::vector<ScalarT>               values(port_count * N);
        std::vector<ScalarT>               derivatives(port_count * N);
        std::vector<IdxT>                  indices(port_count * N);
        std::vector<ScalarT>               read_values(port_count * N);
        std::vector<ScalarT>               read_derivatives(port_count * N);
        std::vector<IdxT>                  read_indices(port_count * N);

        for (std::size_t port = 0; port < port_count; ++port)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            const std::size_t offset = port * N + n;
            values[offset]           = ScalarT{10.0 + static_cast<ScalarT>(offset)};
            derivatives[offset]      = ScalarT{20.0 + static_cast<ScalarT>(offset)};
            indices[offset]          = static_cast<IdxT>(30 + offset);
            input_node_ptrs[port][n] = &input_nodes[offset];
            input_nodes[offset].set(&values[offset], &derivatives[offset], &indices[offset], true);
          }

          signals.template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(
              port,
              input_node_ptrs[port]);
          signals.template readExternalVariables<ComponentSignalsTestExternalVariables::I>(
              port,
              read_values.data() + port * N);
          signals.template readExternalVariableDerivatives<ComponentSignalsTestExternalVariables::I>(
              port,
              read_derivatives.data() + port * N);
          signals.template readExternalVariableIndices<ComponentSignalsTestExternalVariables::I>(
              port,
              read_indices.data() + port * N);
        }

        success *= signals.portSize() == N;
        success *= signals.template externalPortCount<ComponentSignalsTestExternalVariables::I>() == port_count;
        success *= signals.template internalPortCount<ComponentSignalsTestInternalVariables::V>() == 1;

        for (std::size_t port = 0; port < port_count; ++port)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            const std::size_t offset  = port * N + n;
            success                  *= signals.template isAttached<ComponentSignalsTestExternalVariables::I>(port, n);
            success                  *= signals.template isLinked<ComponentSignalsTestExternalVariables::I>(port, n);
            success                  *= signals.template isExternalVariableDifferential<ComponentSignalsTestExternalVariables::I>(
                port,
                n);
            success *= isEqual(
                signals.template readExternalVariable<ComponentSignalsTestExternalVariables::I>(port, n),
                values[offset]);
            success *= isEqual(read_values[offset], values[offset]);
            success *= isEqual(read_derivatives[offset], derivatives[offset]);
            success *= read_indices[offset] == indices[offset];
          }
        }

        signals.template writeExternalVariable<ComponentSignalsTestExternalVariables::I>(1, 2, ScalarT{99.0});
        success *= isEqual(values[1 * N + 2], ScalarT{99.0});

        return success.report(__func__);
      }

      TestOutcome repeatedInternalPorts()
      {
        TestStatus success = true;

        constexpr std::size_t N          = 3;
        constexpr std::size_t port_count = 2;

        SignalsT signals(N);
        signals.template setInternalPortCount<ComponentSignalsTestInternalVariables::V>(port_count);

        std::vector<SignalT>               output_nodes(port_count * N);
        std::vector<std::vector<SignalT*>> output_node_ptrs(port_count, std::vector<SignalT*>(N));

        for (std::size_t port = 0; port < port_count; ++port)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            const std::size_t offset  = port * N + n;
            output_node_ptrs[port][n] = &output_nodes[offset];
          }

          signals.template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(
              port,
              output_node_ptrs[port]);
        }

        success *= signals.portSize() == N;
        success *= signals.template internalPortCount<ComponentSignalsTestInternalVariables::V>() == port_count;
        success *= signals.template externalPortCount<ComponentSignalsTestExternalVariables::I>() == 1;

        for (std::size_t port = 0; port < port_count; ++port)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            const std::size_t offset  = port * N + n;
            success                  *= signals.template isAssigned<ComponentSignalsTestInternalVariables::V>(port, n);
            success                  *= signals.template getSignalNode<ComponentSignalsTestInternalVariables::V>(port, n)
                       == &output_nodes[offset];
          }
        }

        return success.report(__func__);
      }

      TestOutcome invalidPorts()
      {
        TestStatus success = true;

        SignalT               node;
        SignalsT              signals(3);
        std::vector<SignalT*> wrong_width{&node, &node};
        std::vector<SignalT*> right_width{&node, &node, &node};

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

        signals.template setExternalPortCount<ComponentSignalsTestExternalVariables::I>(2);
        signals.template setInternalPortCount<ComponentSignalsTestInternalVariables::V>(2);

        success *= throws<std::logic_error>(
            [&]()
            { signals.template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(0, wrong_width); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(0, wrong_width); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template attachSignalNode<ComponentSignalsTestExternalVariables::I>(2, 0, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template assignSignalNode<ComponentSignalsTestInternalVariables::V>(2, 0, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template attachSignalNode<ComponentSignalsTestExternalVariables::I>(0, 3, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template assignSignalNode<ComponentSignalsTestInternalVariables::V>(0, 3, &node); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template readExternalVariable<ComponentSignalsTestExternalVariables::I>(2, 0); });

        signals.template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(0, right_width);
        signals.template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(0, right_width);

        success *= throws<std::logic_error>(
            [&]()
            { signals.template setExternalPortCount<ComponentSignalsTestExternalVariables::I>(3); });
        success *= throws<std::logic_error>(
            [&]()
            { signals.template setInternalPortCount<ComponentSignalsTestInternalVariables::V>(3); });

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

      TestOutcome repeatedPortComponentUsage()
      {
        TestStatus success = true;

        constexpr std::size_t                            N           = 3;
        constexpr std::size_t                            input_count = 2;
        ComponentSignalsRepeatedPortModel<ScalarT, IdxT> model(N, input_count);

        std::vector<SignalT>               v_nodes(N);
        std::vector<SignalT*>              v_node_ptrs(N);
        std::vector<SignalT>               i_nodes(input_count * N);
        std::vector<std::vector<SignalT*>> i_node_ptrs(input_count, std::vector<SignalT*>(N));
        std::vector<ScalarT>               currents(input_count * N);
        std::vector<ScalarT>               current_derivatives(input_count * N);
        std::vector<IdxT>                  current_indices(input_count * N);

        for (std::size_t n = 0; n < N; ++n)
        {
          v_node_ptrs[n] = &v_nodes[n];
        }

        for (std::size_t port = 0; port < input_count; ++port)
        {
          for (std::size_t n = 0; n < N; ++n)
          {
            const std::size_t offset    = port * N + n;
            currents[offset]            = ScalarT{10.0 * static_cast<ScalarT>(port + 1)
                                       + static_cast<ScalarT>(n)};
            current_derivatives[offset] = ScalarT{1.0 + static_cast<ScalarT>(offset)};
            current_indices[offset]     = static_cast<IdxT>(100 + offset);
            i_node_ptrs[port][n]        = &i_nodes[offset];
            i_nodes[offset].set(&currents[offset], &current_derivatives[offset], &current_indices[offset], true);
          }
        }

        model.getSignals().template assignSignalNodes<ComponentSignalsTestInternalVariables::V>(v_node_ptrs);
        for (std::size_t port = 0; port < input_count; ++port)
        {
          model.getSignals().template attachSignalNodes<ComponentSignalsTestExternalVariables::I>(
              port,
              i_node_ptrs[port]);
        }

        model.allocate();

        for (std::size_t n = 0; n < N; ++n)
        {
          model.setLocal(n, ScalarT{100.0 + static_cast<ScalarT>(n)}, ScalarT{200.0 + static_cast<ScalarT>(n)});
        }

        model.evaluateResidual();

        const auto& residual = model.getResidual();
        const auto& values   = model.inputValues();
        const auto& indices  = model.inputIndices();

        for (std::size_t n = 0; n < N; ++n)
        {
          ScalarT expected_residual = ScalarT{-100.0 - static_cast<ScalarT>(n)};
          for (std::size_t port = 0; port < input_count; ++port)
          {
            expected_residual += currents[port * N + n];
          }

          success *= isEqual(v_nodes[n].read(), ScalarT{100.0 + static_cast<ScalarT>(n)});
          success *= isEqual(v_nodes[n].readDerivative(), ScalarT{200.0 + static_cast<ScalarT>(n)});
          success *= v_nodes[n].getVariableIndex() == static_cast<IdxT>(n);
          success *= isEqual(residual[n], expected_residual);
        }

        for (std::size_t offset = 0; offset < input_count * N; ++offset)
        {
          success *= isEqual(values[offset], currents[offset]);
          success *= indices[offset] == current_indices[offset];
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
