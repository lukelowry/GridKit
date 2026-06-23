/**
 * @file SignalNode model implementation.
 */
#include <stdexcept>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNodeData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    template <typename scalar_type, typename index_type>
    SignalNode<scalar_type, index_type>::SignalNode()
    {
    }

    template <typename scalar_type, typename index_type>
    SignalNode<scalar_type, index_type>::SignalNode(const SignalNodeData<RealT, IdxT>& data)
      : signal_id_(data.signal_id)
    {
    }

    template <typename scalar_type, typename index_type>
    void SignalNode<scalar_type, index_type>::set(ScalarT* signal, IdxT* variable_index)
    {
      set(signal, nullptr, variable_index, false);
    }

    template <typename scalar_type, typename index_type>
    void SignalNode<scalar_type, index_type>::set(ScalarT* signal,
                                                  ScalarT* signal_derivative,
                                                  IdxT*    variable_index,
                                                  bool     differential)
    {
      signal_            = signal;
      signal_derivative_ = signal_derivative;
      variable_index_    = variable_index;
      differential_      = differential;
    }

    template <typename scalar_type, typename index_type>
    bool SignalNode<scalar_type, index_type>::linked() const
    {
      return (signal_) && (variable_index_);
    }

    template <typename scalar_type, typename index_type>
    scalar_type SignalNode<scalar_type, index_type>::read() const
    {
      return *signal_;
    }

    template <typename scalar_type, typename index_type>
    scalar_type SignalNode<scalar_type, index_type>::readDerivative() const
    {
      if (!isDifferential() || signal_derivative_ == nullptr)
      {
        throw std::logic_error("Signal derivative requested from a non-differential signal node");
      }

      return *signal_derivative_;
    }

    template <typename scalar_type, typename index_type>
    bool SignalNode<scalar_type, index_type>::isDifferential() const
    {
      return differential_;
    }

    template <typename scalar_type, typename index_type>
    void SignalNode<scalar_type, index_type>::init(ScalarT signal)
    {
      *signal_ = signal;
    }
  } // namespace PhasorDynamics
} // namespace GridKit
