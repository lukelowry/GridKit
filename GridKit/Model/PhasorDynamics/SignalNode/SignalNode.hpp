#pragma once

#include <GridKit/Constants.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    template <typename real_type, typename index_type>
    struct SignalNodeData;
  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    /*!
     * @brief SignalNode model implementation base class.
     *
     */
    template <typename scalar_type, typename index_type>
    class SignalNode
    {
    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using RealT   = typename GridKit::ScalarTraits<ScalarT>::RealT;

      SignalNode();
      SignalNode(const SignalNodeData<RealT, IdxT>& data);

      virtual ~SignalNode() = default;

      void    set(ScalarT* signal_in, IdxT* global_index);
      void    set(ScalarT* signal_in, ScalarT* signal_derivative_in, IdxT* global_index, bool differential);
      bool    linked() const;
      bool    isDifferential() const;
      ScalarT read() const;
      ScalarT readDerivative() const;
      void    init(ScalarT signal_in);

      const IdxT signalId() const
      {
        return signal_id_;
      }

      IdxT getVariableIndex() const
      {
        return *variable_index_;
      }

      virtual const IdxT busID() const
      {
        return bus_id_;
      }

    private:
      ScalarT* signal_{nullptr};
      ScalarT* signal_derivative_{nullptr};
      IdxT     signal_id_{0};
      bool     differential_{false};

    protected:
      const IdxT bus_id_{INVALID_INDEX<IdxT>};

      IdxT* variable_index_{nullptr};
    };

  } // namespace PhasorDynamics
} // namespace GridKit
