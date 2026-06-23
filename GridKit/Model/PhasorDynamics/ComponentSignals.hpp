#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    /// Dummy `Variables` type for components with no variables
    enum class NoVariables : size_t
    {
      MAXIMUM
    };

    /// Concept requiring an enum to have a `MAXIMUM` variant and that it have
    /// the underlying type of `size_t`. This does not ensure the variant is
    /// the actual maximum
    template <typename T>
    concept EnumHasMaximumValueAndIsSizeT = std::is_enum_v<T>
                                            && std::is_same_v<std::underlying_type_t<T>, size_t>
                                            && requires { T::MAXIMUM; };

    /// Extension object for `Component`s adding methods and member variables
    /// related to signal bus management
    ///
    /// This is used by adding an instance in a field to your class and
    /// exposing this field to others
    ///
    /// @tparam scalar_type Scalar value type
    /// @tparam index_type Index type
    /// @tparam InternalVariables An enumeration satisfying
    ///         `EnumHasMaximumValueAndIsSizeT` enumerating internal variables
    ///         for the component
    /// @tparam ExternalVariables An enumeration satisfying
    ///         `EnumHasMaximumValueAndIsSizeT` enumerating external variables
    ///         for the component
    /// @invariant InternalVariables::MAXIMUM is the greatest attainable
    ///            integer value of the enum
    /// @invariant ExternalVariables::MAXIMUM is the greatest attainable
    ///            integer value of the enum
    template <typename scalar_type, typename index_type, typename InternalVariables, typename ExternalVariables>
      requires EnumHasMaximumValueAndIsSizeT<InternalVariables>
               && EnumHasMaximumValueAndIsSizeT<ExternalVariables>
    class ComponentSignals
    {
    public:
      /// Scalar value type
      using ScalarT = scalar_type;
      /// Index type
      using IdxT    = index_type;
      /// Signal node type
      using SignalT = SignalNode<ScalarT, IdxT>;

      explicit ComponentSignals(std::size_t port_size = 1)
        : port_size_(port_size)
      {
        if (port_size_ == 0)
        {
          throw std::logic_error("Signal port size must be positive");
        }

        resizeSlots(internal_variable_signals_);
        resizeSlots(external_variable_signals_);
      }

      std::size_t portSize() const
      {
        return port_size_;
      }

      /// Sets the number of ports for an external variable
      ///
      /// @tparam variable The external variable to resize
      /// @param[in] count Number of ports for the variable
      /// @pre No signal nodes have been attached for this variable when
      ///      changing the port count
      template <ExternalVariables variable>
      auto setExternalPortCount(std::size_t count)
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        setPortCount(external_variable_signals_[static_cast<std::size_t>(variable)], count);
      }

      /// Sets the number of ports for an internal variable
      ///
      /// @tparam variable The internal variable to resize
      /// @param[in] count Number of ports for the variable
      /// @pre No signal nodes have been assigned for this variable when
      ///      changing the port count
      template <InternalVariables variable>
      auto setInternalPortCount(std::size_t count)
      {
        static_assert(variable < InternalVariables::MAXIMUM);
        setPortCount(internal_variable_signals_[static_cast<std::size_t>(variable)], count);
      }

      /// Returns the number of ports for an external variable
      ///
      /// @tparam variable The external variable to query
      template <ExternalVariables variable>
      auto externalPortCount() const -> std::size_t
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return external_variable_signals_[static_cast<std::size_t>(variable)].size();
      }

      /// Returns the number of ports for an internal variable
      ///
      /// @tparam variable The internal variable to query
      template <InternalVariables variable>
      auto internalPortCount() const -> std::size_t
      {
        static_assert(variable < InternalVariables::MAXIMUM);
        return internal_variable_signals_[static_cast<std::size_t>(variable)].size();
      }

      /// Attaches a signal node to an external variable on this component
      ///
      /// @tparam variable The external variable to attach the provided
      ///         signal to
      /// @param[in] node The signal node to attach
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is attached to the indicated
      ///       external variable
      template <ExternalVariables variable>
      auto attachSignalNode(SignalT* node)
      {
        attachSignalNode<variable>(0, node);
      }

      /// Attaches a signal node to one element of an external variable port
      ///
      /// @tparam variable The external variable to attach the provided
      ///         signal to
      /// @param[in] n Port element index
      /// @param[in] node The signal node to attach
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is attached to the indicated
      ///       external variable element
      template <ExternalVariables variable>
      auto attachSignalNode(std::size_t n, SignalT* node)
      {
        attachSignalNode<variable>(0, n, node);
      }

      /// Attaches a signal node to one element of an external variable port
      ///
      /// @tparam variable The external variable to attach the provided
      ///         signal to
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @param[in] node The signal node to attach
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is attached to the indicated
      ///       external variable port element
      template <ExternalVariables variable>
      auto attachSignalNode(std::size_t port, std::size_t n, SignalT* node)
      {
#ifndef NDEBUG
        if (node == nullptr)
        {
          throw std::logic_error("A null pointer to a signal node has been passed to attachSignalNode");
        }
#endif

        static_assert(variable < ExternalVariables::MAXIMUM);
        auto& variable_ports = external_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        variable_ports[port][n] = node;
      }

      /// Attaches signal nodes to all elements of an external variable port
      ///
      /// @tparam variable The external variable to attach the provided
      ///         signals to
      /// @param[in] nodes Signal nodes to attach
      /// @pre `nodes.size()` equals `portSize()`
      /// @post Each signal node is attached to the corresponding port element
      template <ExternalVariables variable>
      auto attachSignalNodes(const std::vector<SignalT*>& nodes)
      {
        attachSignalNodes<variable>(0, nodes);
      }

      /// Attaches signal nodes to all elements of an external variable port
      ///
      /// @tparam variable The external variable to attach the provided
      ///         signals to
      /// @param[in] port Port index
      /// @param[in] nodes Signal nodes to attach
      /// @pre `nodes.size()` equals `portSize()`
      /// @post Each signal node is attached to the corresponding port element
      template <ExternalVariables variable>
      auto attachSignalNodes(std::size_t port, const std::vector<SignalT*>& nodes)
      {
        checkPortSize(nodes.size());
        for (std::size_t n = 0; n < port_size_; ++n)
        {
          attachSignalNode<variable>(port, n, nodes[n]);
        }
      }

      /// Check if a signal node has been attached to an external variable
      ///
      /// @tparam variable The external variable to check
      template <ExternalVariables variable>
      auto isAttached() const -> bool
      {
        return isAttached<variable>(0);
      }

      /// Check if a signal node has been attached to one element of an
      /// external variable port
      ///
      /// @tparam variable The external variable to check
      /// @param[in] n Port element index
      template <ExternalVariables variable>
      auto isAttached(std::size_t n) const -> bool
      {
        return isAttached<variable>(0, n);
      }

      /// Check if a signal node has been attached to one element of an
      /// external variable port
      ///
      /// @tparam variable The external variable to check
      /// @param[in] port Port index
      /// @param[in] n Port element index
      template <ExternalVariables variable>
      auto isAttached(std::size_t port, std::size_t n) const -> bool
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        const auto& variable_ports = external_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        return static_cast<bool>(variable_ports[port][n]);
      }

      /// Check if a signal node has been assigned to an internal variable
      ///
      /// @tparam variable The internal variable to check
      template <InternalVariables variable>
      auto isAssigned() const -> bool
      {
        return isAssigned<variable>(0);
      }

      /// Check if a signal node has been assigned to one element of an
      /// internal variable port
      ///
      /// @tparam variable The internal variable to check
      /// @param[in] n Port element index
      template <InternalVariables variable>
      auto isAssigned(std::size_t n) const -> bool
      {
        return isAssigned<variable>(0, n);
      }

      /// Check if a signal node has been assigned to one element of an
      /// internal variable port
      ///
      /// @tparam variable The internal variable to check
      /// @param[in] port Port index
      /// @param[in] n Port element index
      template <InternalVariables variable>
      auto isAssigned(std::size_t port, std::size_t n) const -> bool
      {
        static_assert(variable < InternalVariables::MAXIMUM);
        const auto& variable_ports = internal_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        return static_cast<bool>(variable_ports[port][n]);
      }

      /// Check if a signal node has been "set"
      ///
      /// @tparam variable The external variable to check
      template <ExternalVariables variable>
      auto isLinked() const -> bool
      {
        return isLinked<variable>(0);
      }

      /// Check if one element of a signal port has been "set"
      ///
      /// @tparam variable The external variable to check
      /// @param[in] n Port element index
      template <ExternalVariables variable>
      auto isLinked(std::size_t n) const -> bool
      {
        return isLinked<variable>(0, n);
      }

      /// Check if one element of a signal port has been "set"
      ///
      /// @tparam variable The external variable to check
      /// @param[in] port Port index
      /// @param[in] n Port element index
      template <ExternalVariables variable>
      auto isLinked(std::size_t port, std::size_t n) const -> bool
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return getExternalSignalNode<variable>(port, n)->linked();
      }

      /// Returns a signal node for an internal signal variable to be
      /// attached to an external variable on another component
      ///
      /// @tparam variable The internal variable to get the assigned
      ///         signal node of
      /// @pre A signal node has been assigned to the requested internal
      ///      variable
      template <InternalVariables variable>
      auto getSignalNode() -> SignalT*
      {
        return getSignalNode<variable>(0);
      }

      /// Returns a signal node for one element of an internal signal
      /// variable port to be attached to an external variable on another
      /// component
      ///
      /// @tparam variable The internal variable to get the assigned
      ///         signal node of
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested internal
      ///      variable element
      template <InternalVariables variable>
      auto getSignalNode(std::size_t n) -> SignalT*
      {
        return getSignalNode<variable>(0, n);
      }

      /// Returns a signal node for one element of an internal signal
      /// variable port to be attached to an external variable on another
      /// component
      ///
      /// @tparam variable The internal variable to get the assigned
      ///         signal node of
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested internal
      ///      variable port element
      template <InternalVariables variable>
      auto getSignalNode(std::size_t port, std::size_t n) -> SignalT*
      {
        static_assert(variable < InternalVariables::MAXIMUM);
        auto& variable_ports = internal_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        if (!variable_ports[port][n])
        {
          throw std::logic_error("A signal node has not been assigned to this internal variable");
        }

        return *variable_ports[port][n];
      }

      /// Returns the value of the specified external variable
      ///
      /// @tparam variable The external variable to read from
      /// @pre A signal node has been assigned to the requested external
      ///      variable
      template <ExternalVariables variable>
      auto readExternalVariable() const -> ScalarT
      {
        return readExternalVariable<variable>(0);
      }

      /// Returns the value of one element of the specified external variable
      /// port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable element
      template <ExternalVariables variable>
      auto readExternalVariable(std::size_t n) const -> ScalarT
      {
        return readExternalVariable<variable>(0, n);
      }

      /// Returns the value of one element of the specified external variable
      /// port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable port element
      template <ExternalVariables variable>
      auto readExternalVariable(std::size_t port, std::size_t n) const -> ScalarT
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return getExternalSignalNode<variable>(port, n)->read();
      }

      /// Returns the values of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[out] values Values read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariables(ScalarT* values) const
      {
        readExternalVariables<variable>(0, values);
      }

      /// Returns the values of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[out] values Values read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariables(std::size_t port, ScalarT* values) const
      {
        for (std::size_t n = 0; n < port_size_; ++n)
        {
          values[n] = readExternalVariable<variable>(port, n);
        }
      }

      /// Returns true if the specified external variable is differential
      ///
      /// @tparam variable The external variable to check
      /// @pre A signal node has been assigned to the requested external
      ///      variable
      template <ExternalVariables variable>
      auto isExternalVariableDifferential() const -> bool
      {
        return isExternalVariableDifferential<variable>(0);
      }

      /// Returns true if one element of the specified external variable port
      /// is differential
      ///
      /// @tparam variable The external variable to check
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable element
      template <ExternalVariables variable>
      auto isExternalVariableDifferential(std::size_t n) const -> bool
      {
        return isExternalVariableDifferential<variable>(0, n);
      }

      /// Returns true if one element of the specified external variable port
      /// is differential
      ///
      /// @tparam variable The external variable to check
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable port element
      template <ExternalVariables variable>
      auto isExternalVariableDifferential(std::size_t port, std::size_t n) const -> bool
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return getExternalSignalNode<variable>(port, n)->isDifferential();
      }

      /// Returns the derivative of the specified external variable
      ///
      /// @tparam variable The external variable to read from
      /// @pre A signal node has been assigned to the requested external
      ///      variable and it is differential
      template <ExternalVariables variable>
      auto readExternalVariableDerivative() const -> ScalarT
      {
        return readExternalVariableDerivative<variable>(0);
      }

      /// Returns the derivative of one element of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable element and it is differential
      template <ExternalVariables variable>
      auto readExternalVariableDerivative(std::size_t n) const -> ScalarT
      {
        return readExternalVariableDerivative<variable>(0, n);
      }

      /// Returns the derivative of one element of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable port element and it is differential
      template <ExternalVariables variable>
      auto readExternalVariableDerivative(std::size_t port, std::size_t n) const -> ScalarT
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return getExternalSignalNode<variable>(port, n)->readDerivative();
      }

      /// Returns the derivatives of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[out] derivatives Derivatives read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariableDerivatives(ScalarT* derivatives) const
      {
        readExternalVariableDerivatives<variable>(0, derivatives);
      }

      /// Returns the derivatives of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[out] derivatives Derivatives read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariableDerivatives(std::size_t port, ScalarT* derivatives) const
      {
        for (std::size_t n = 0; n < port_size_; ++n)
        {
          derivatives[n] = readExternalVariableDerivative<variable>(port, n);
        }
      }

      /// Returns the global index of the specified external variable
      ///
      /// @tparam variable The external variable to read from
      /// @pre A signal node has been assigned to the requested external
      ///      variable
      template <ExternalVariables variable>
      auto readExternalVariableIndex() const -> IdxT
      {
        return readExternalVariableIndex<variable>(0);
      }

      /// Returns the global index of one element of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable element
      template <ExternalVariables variable>
      auto readExternalVariableIndex(std::size_t n) const -> IdxT
      {
        return readExternalVariableIndex<variable>(0, n);
      }

      /// Returns the global index of one element of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @pre A signal node has been assigned to the requested external
      ///      variable port element
      template <ExternalVariables variable>
      auto readExternalVariableIndex(std::size_t port, std::size_t n) const -> IdxT
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        return getExternalSignalNode<variable>(port, n)->getVariableIndex();
      }

      /// Returns the global indices of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[out] indices Global indices read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariableIndices(IdxT* indices) const
      {
        readExternalVariableIndices<variable>(0, indices);
      }

      /// Returns the global indices of all elements of the specified external
      /// variable port
      ///
      /// @tparam variable The external variable to read from
      /// @param[in] port Port index
      /// @param[out] indices Global indices read from the port elements
      template <ExternalVariables variable>
      auto readExternalVariableIndices(std::size_t port, IdxT* indices) const
      {
        for (std::size_t n = 0; n < port_size_; ++n)
        {
          indices[n] = readExternalVariableIndex<variable>(port, n);
        }
      }

      /// Writes a value to the specified external variable
      ///
      /// @warning This method should be used only in component initialization
      /// methods. Use only if you know what you are doing.
      ///
      /// @tparam variable The external variable to write to
      /// @param[in] value The value to write to the signal node
      /// @pre A signal node has been assigned to the requested external
      ///      variable
      /// @post The signal node of the corresponding external variable has
      ///       the given value written to it
      template <ExternalVariables variable>
      auto writeExternalVariable(ScalarT value)
      {
        writeExternalVariable<variable>(0, value);
      }

      /// Writes a value to one element of the specified external variable port
      ///
      /// @warning This method should be used only in component initialization
      /// methods. Use only if you know what you are doing.
      ///
      /// @tparam variable The external variable to write to
      /// @param[in] n Port element index
      /// @param[in] value The value to write to the signal node
      /// @pre A signal node has been assigned to the requested external
      ///      variable element
      /// @post The signal node of the corresponding external variable element
      ///       has the given value written to it
      template <ExternalVariables variable>
      auto writeExternalVariable(std::size_t n, ScalarT value)
      {
        writeExternalVariable<variable>(0, n, value);
      }

      /// Writes a value to one element of the specified external variable port
      ///
      /// @warning This method should be used only in component initialization
      /// methods. Use only if you know what you are doing.
      ///
      /// @tparam variable The external variable to write to
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @param[in] value The value to write to the signal node
      /// @pre A signal node has been assigned to the requested external
      ///      variable port element
      /// @post The signal node of the corresponding external variable port
      ///       element has the given value written to it
      template <ExternalVariables variable>
      auto writeExternalVariable(std::size_t port, std::size_t n, ScalarT value)
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        getExternalSignalNode<variable>(port, n)->init(value);
      }

      /// Assigns a signal node to an internal variable on this component
      ///
      /// @tparam variable The internal variable to assign the signal node to
      /// @param[in] node The signal node to assign
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is assigned to the indicated
      ///       internal variable
      template <InternalVariables variable>
      auto assignSignalNode(SignalT* node)
      {
        assignSignalNode<variable>(0, node);
      }

      /// Assigns a signal node to one element of an internal variable port on
      /// this component
      ///
      /// @tparam variable The internal variable to assign the signal node to
      /// @param[in] n Port element index
      /// @param[in] node The signal node to assign
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is assigned to the indicated internal
      ///       variable element
      template <InternalVariables variable>
      auto assignSignalNode(std::size_t n, SignalT* node)
      {
        assignSignalNode<variable>(0, n, node);
      }

      /// Assigns a signal node to one element of an internal variable port on
      /// this component
      ///
      /// @tparam variable The internal variable to assign the signal node to
      /// @param[in] port Port index
      /// @param[in] n Port element index
      /// @param[in] node The signal node to assign
      /// @pre The provided pointer to a signal node is not `nullptr`
      /// @post The provided signal node is assigned to the indicated internal
      ///       variable port element
      template <InternalVariables variable>
      auto assignSignalNode(std::size_t port, std::size_t n, SignalT* node)
      {
#ifndef NDEBUG
        if (node == nullptr)
        {
          throw std::logic_error("A null pointer to a signal node has been passed to assignSignalNode");
        }
#endif

        static_assert(variable < InternalVariables::MAXIMUM);
        auto& variable_ports = internal_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        variable_ports[port][n] = node;
      }

      /// Assigns signal nodes to all elements of an internal variable port on
      /// this component
      ///
      /// @tparam variable The internal variable to assign the signal nodes to
      /// @param[in] nodes Signal nodes to assign
      /// @pre `nodes.size()` equals `portSize()`
      /// @post Each signal node is assigned to the corresponding port element
      template <InternalVariables variable>
      auto assignSignalNodes(const std::vector<SignalT*>& nodes)
      {
        assignSignalNodes<variable>(0, nodes);
      }

      /// Assigns signal nodes to all elements of an internal variable port on
      /// this component
      ///
      /// @tparam variable The internal variable to assign the signal nodes to
      /// @param[in] port Port index
      /// @param[in] nodes Signal nodes to assign
      /// @pre `nodes.size()` equals `portSize()`
      /// @post Each signal node is assigned to the corresponding port element
      template <InternalVariables variable>
      auto assignSignalNodes(std::size_t port, const std::vector<SignalT*>& nodes)
      {
        checkPortSize(nodes.size());
        for (std::size_t n = 0; n < port_size_; ++n)
        {
          assignSignalNode<variable>(port, n, nodes[n]);
        }
      }

    private:
      using SignalPort          = std::vector<std::optional<SignalT*>>;
      using VariableSignalPorts = std::vector<SignalPort>;

      template <typename Variables>
      using SignalSlots =
          std::array<VariableSignalPorts,
                     static_cast<std::size_t>(Variables::MAXIMUM)>;

      template <typename Slots>
      void resizeSlots(Slots& slots)
      {
        for (auto& variable_ports : slots)
        {
          resizePortCount(variable_ports, 1);
        }
      }

      void resizePortCount(VariableSignalPorts& ports, std::size_t count)
      {
        ports.resize(count);
        for (auto& port : ports)
        {
          port.resize(port_size_);
        }
      }

      void setPortCount(VariableSignalPorts& ports, std::size_t count)
      {
        if (count != ports.size() && hasSignalNode(ports))
        {
          throw std::logic_error(
              "Signal port count cannot be changed after signal nodes have been attached or assigned");
        }

        resizePortCount(ports, count);
      }

      auto hasSignalNode(const VariableSignalPorts& ports) const -> bool
      {
        for (const auto& port : ports)
        {
          for (const auto& node : port)
          {
            if (node)
            {
              return true;
            }
          }
        }

        return false;
      }

      void checkPort(std::size_t port, const VariableSignalPorts& ports) const
      {
        if (port >= ports.size())
        {
          throw std::logic_error("Signal port index out of range");
        }
      }

      void checkIndex(std::size_t n) const
      {
        if (n >= port_size_)
        {
          throw std::logic_error("Signal port element index out of range");
        }
      }

      void checkPortSize(std::size_t size) const
      {
        if (size != port_size_)
        {
          throw std::logic_error("Signal port width mismatch");
        }
      }

      template <ExternalVariables variable>
      auto getExternalSignalNode(std::size_t port, std::size_t n) const -> SignalT*
      {
        static_assert(variable < ExternalVariables::MAXIMUM);
        const auto& variable_ports = external_variable_signals_[static_cast<std::size_t>(variable)];
        checkPort(port, variable_ports);
        checkIndex(n);
        if (!variable_ports[port][n])
        {
          throw std::logic_error("A signal node has not been assigned to this external variable");
        }

        return *variable_ports[port][n];
      }

      std::size_t port_size_{1};

      /// Internal variables which may have a signal associated with them for
      /// use elsewhere
      SignalSlots<InternalVariables> internal_variable_signals_{};

      /// External variables which may have a signal associated with them for
      /// use internally
      SignalSlots<ExternalVariables> external_variable_signals_{};
    };
  } // namespace PhasorDynamics
} // namespace GridKit
