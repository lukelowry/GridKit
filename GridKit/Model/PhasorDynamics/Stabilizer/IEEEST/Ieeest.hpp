/**
 * @file Ieeest.hpp
 * @author Luke Lowery (lukel@tamu.edu)
 * @brief Declaration of the IEEEST Power System Stabilizer.
 */

#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Stabilizer
    {
      template <typename real_type, typename index_type>
      struct IeeestData;
    } // namespace Stabilizer

    template <typename scalar_type, typename index_type>
    class SignalNode;

  } // namespace PhasorDynamics
} // namespace GridKit

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Stabilizer
    {
      /**
       * @brief Combined denominator coefficients of the IEEEST notch filter.
       *
       * The two second-order denominator factors expand into a single quartic
       * with coefficients a1..a4.
       */
      template <typename real_type>
      inline std::array<real_type, 4> notchCoefficients(real_type A1,
                                                        real_type A2,
                                                        real_type A3,
                                                        real_type A4)
      {
        return {A1 + A3,
                A2 + A4 + A1 * A3,
                A1 * A4 + A2 * A3,
                A2 * A4};
      }

      /**
       * @brief Notch-filter order implied by the combined denominator
       *        coefficients.
       *
       * Structural coefficients are exact data-file literals, so the
       * comparisons against zero are intentionally exact.
       */
      template <typename real_type>
      inline size_t notchOrder(real_type a1, real_type a2, real_type a3, real_type a4)
      {
        size_t order = 0;

        if (a1 != ZERO<real_type>)
        {
          order = 1;
        }
        if (a2 != ZERO<real_type>)
        {
          order = 2;
        }
        if (a3 != ZERO<real_type>)
        {
          order = 3;
        }
        if (a4 != ZERO<real_type>)
        {
          order = 4;
        }

        return order;
      }

      /// Internal variable layout of a `Ieeest` by notch-filter order
      template <size_t order>
      struct IeeestVariables;

      template <>
      struct IeeestVariables<0>
      {
        /// Internal variables of a zeroth-order `Ieeest`
        enum class InternalVariables : size_t
        {
          X5,  ///< Lead-lag 1 state
          X6,  ///< Lead-lag 2 state
          X7,  ///< Washout state
          V4,  ///< Notch filter output
          V5,  ///< Lead-lag 1 output
          V6,  ///< Lead-lag 2 output
          V7,  ///< Unlimited stabilizer signal
          VSS, ///< Limited stabilizer signal (model output)
          MAXIMUM,
        };
      };

      template <>
      struct IeeestVariables<1>
      {
        /// Internal variables of a first-order `Ieeest`
        enum class InternalVariables : size_t
        {
          X1,  ///< Notch filter state 1
          X5,  ///< Lead-lag 1 state
          X6,  ///< Lead-lag 2 state
          X7,  ///< Washout state
          V4,  ///< Notch filter output
          V5,  ///< Lead-lag 1 output
          V6,  ///< Lead-lag 2 output
          V7,  ///< Unlimited stabilizer signal
          VSS, ///< Limited stabilizer signal (model output)
          MAXIMUM,
        };
      };

      template <>
      struct IeeestVariables<2>
      {
        /// Internal variables of a second-order `Ieeest`
        enum class InternalVariables : size_t
        {
          X1,  ///< Notch filter state 1
          X2,  ///< Notch filter state 2
          X5,  ///< Lead-lag 1 state
          X6,  ///< Lead-lag 2 state
          X7,  ///< Washout state
          V4,  ///< Notch filter output
          V5,  ///< Lead-lag 1 output
          V6,  ///< Lead-lag 2 output
          V7,  ///< Unlimited stabilizer signal
          VSS, ///< Limited stabilizer signal (model output)
          MAXIMUM,
        };
      };

      template <>
      struct IeeestVariables<3>
      {
        /// Internal variables of a third-order `Ieeest`
        enum class InternalVariables : size_t
        {
          X1,  ///< Notch filter state 1
          X2,  ///< Notch filter state 2
          X3,  ///< Notch filter state 3
          X5,  ///< Lead-lag 1 state
          X6,  ///< Lead-lag 2 state
          X7,  ///< Washout state
          V4,  ///< Notch filter output
          V5,  ///< Lead-lag 1 output
          V6,  ///< Lead-lag 2 output
          V7,  ///< Unlimited stabilizer signal
          VSS, ///< Limited stabilizer signal (model output)
          MAXIMUM,
        };
      };

      template <>
      struct IeeestVariables<4>
      {
        /// Internal variables of a fourth-order `Ieeest`
        enum class InternalVariables : size_t
        {
          X1,  ///< Notch filter state 1
          X2,  ///< Notch filter state 2
          X3,  ///< Notch filter state 3
          X4,  ///< Notch filter state 4
          X5,  ///< Lead-lag 1 state
          X6,  ///< Lead-lag 2 state
          X7,  ///< Washout state
          V4,  ///< Notch filter output
          V5,  ///< Lead-lag 1 output
          V6,  ///< Lead-lag 2 output
          V7,  ///< Unlimited stabilizer signal
          VSS, ///< Limited stabilizer signal (model output)
          MAXIMUM,
        };
      };

      /// Internal variables of a `Ieeest` of the given notch-filter order
      template <size_t order>
      using IeeestInternalVariables = typename IeeestVariables<order>::InternalVariables;

      /// External variables of a `Ieeest`
      enum class IeeestExternalVariables : size_t
      {
        U, ///< Stabilizer input signal
        MAXIMUM,
      };

      template <typename scalar_type, typename index_type, size_t order>
      class Ieeest : public Component<scalar_type, index_type>
      {
        static_assert(order <= 4, "Ieeest notch filter order must be in [0, 4]");

        using Component<scalar_type, index_type>::gridkit_component_id_;
        using Component<scalar_type, index_type>::alpha_;
        using Component<scalar_type, index_type>::f_;
        using Component<scalar_type, index_type>::nnz_;
        using Component<scalar_type, index_type>::size_;
        using Component<scalar_type, index_type>::tag_;
        using Component<scalar_type, index_type>::abs_tol_;
        using Component<scalar_type, index_type>::time_;
        using Component<scalar_type, index_type>::y_;
        using Component<scalar_type, index_type>::yp_;
        using Component<scalar_type, index_type>::wb_;
        using Component<scalar_type, index_type>::J_rows_buffer_;
        using Component<scalar_type, index_type>::J_cols_buffer_;
        using Component<scalar_type, index_type>::J_vals_buffer_;
        using Component<scalar_type, index_type>::variable_indices_;
        using Component<scalar_type, index_type>::residual_indices_;
        using Component<scalar_type, index_type>::allocated_;

      public:
        using ScalarT    = scalar_type;
        using IdxT       = index_type;
        using RealT      = typename Component<ScalarT, IdxT>::RealT;
        using ModelDataT = IeeestData<RealT, IdxT>;
        using SignalT    = SignalNode<ScalarT, IdxT>;
        using MonitorT   = Model::VariableMonitor<Ieeest, IeeestData>;

        Ieeest();
        Ieeest(const ModelDataT& data);
        ~Ieeest();

        int setGridKitComponentID(IdxT) override final;
        int allocate() override final;
        int verify() const override final;
        int initialize() override final;
        int tagDifferentiable() override final;
        int setAbsoluteTolerance(RealT rel_tol) override final;
        int evaluateResidual() override final;
        int evaluateJacobian() override final;

        /// Get the `ComponentSignals` from this `Ieeest`
        auto getSignals()
            -> ComponentSignals<ScalarT,
                                IdxT,
                                IeeestInternalVariables<order>,
                                IeeestExternalVariables>&
        {
          return signals_;
        }

        const Model::VariableMonitorBase* getMonitor() const override;

        __attribute__((always_inline)) inline int evaluateInternalResidual(
            const ScalarT*,
            const ScalarT*,
            const ScalarT*,
            const ScalarT*,
            ScalarT*);

      private:
        static constexpr RealT TIME_CONSTANT_MINIMUM = static_cast<RealT>(1.0e-3);

        RealT A1_{0};
        RealT A2_{0};
        RealT A3_{0};
        RealT A4_{0};
        RealT A5_{0};
        RealT A6_{0};
        RealT T1_{0};
        RealT T2_{1};
        RealT T3_{0};
        RealT T4_{1};
        RealT T5_{0};
        RealT T6_{1};
        RealT Ks_{1};
        RealT Lsmin_{-0.1};
        RealT Lsmax_{0.1};
        RealT Vcl_{0};
        RealT Vcu_{0};
        RealT Tdelay_{0};

        RealT a1_{0};
        RealT a2_{0};
        RealT a3_{0};
        RealT a4_{0};

        IdxT parameter_error_count_{0};

        ComponentSignals<ScalarT, IdxT, IeeestInternalVariables<order>, IeeestExternalVariables> signals_;

        std::unique_ptr<MonitorT> monitor_;

        void initializeParameters(const ModelDataT& data);
        void initializeMonitor();
        void setDerivedParameters();

        std::vector<ScalarT> ws_;
        std::vector<IdxT>    ws_indices_;
      };

    } // namespace Stabilizer
  } // namespace PhasorDynamics
} // namespace GridKit
