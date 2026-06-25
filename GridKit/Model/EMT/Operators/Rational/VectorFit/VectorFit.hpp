#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/Component.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>
#include <GridKit/Model/PhasorDynamics/VectorizedEquations.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VectorFitInternalVariables : size_t
    {
      Y_OUT,
      MAXIMUM,
    };

    enum class VectorFitExternalVariables : size_t
    {
      INPUT,
      MAXIMUM,
    };

    template <typename scalar_type, typename index_type, std::size_t N, std::size_t K>
    class VectorFit final : public EMT::Component<scalar_type, index_type>
    {
      using EMT::Component<scalar_type, index_type>::gridkit_component_id_;
      using EMT::Component<scalar_type, index_type>::size_;
      using EMT::Component<scalar_type, index_type>::nnz_;
      using EMT::Component<scalar_type, index_type>::y_;
      using EMT::Component<scalar_type, index_type>::yp_;
      using EMT::Component<scalar_type, index_type>::f_;
      using EMT::Component<scalar_type, index_type>::tag_;
      using EMT::Component<scalar_type, index_type>::abs_tol_;
      using EMT::Component<scalar_type, index_type>::wb_;
      using EMT::Component<scalar_type, index_type>::J_;
      using EMT::Component<scalar_type, index_type>::J_rows_buffer_;
      using EMT::Component<scalar_type, index_type>::J_cols_buffer_;
      using EMT::Component<scalar_type, index_type>::J_vals_buffer_;
      using EMT::Component<scalar_type, index_type>::variable_indices_;
      using EMT::Component<scalar_type, index_type>::residual_indices_;
      using EMT::Component<scalar_type, index_type>::alpha_;
      using EMT::Component<scalar_type, index_type>::offset_;
      using EMT::Component<scalar_type, index_type>::allocated_;
      using EMT::Component<scalar_type, index_type>::allocateVectors;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using BaseT      = EMT::Component<ScalarT, IdxT>;
      using RealT      = typename BaseT::RealT;
      using MatrixT    = PhasorDynamics::Equation::Mat<RealT, N, K>;
      using ModelDataT = VectorFitData<RealT, IdxT, N, K>;
      using PoleT      = typename ModelDataT::PoleT;
      using SignalsT   = PhasorDynamics::ComponentSignals<ScalarT,
                                                          IdxT,
                                                          VectorFitInternalVariables,
                                                          VectorFitExternalVariables>;

      static_assert(N > 0, "VectorFit output dimension must be positive");
      static_assert(K > 0, "VectorFit input dimension must be positive");

      VectorFit();
      explicit VectorFit(const ModelDataT& data);
      ~VectorFit() override = default;

      auto getSignals() -> SignalsT&
      {
        return signals_;
      }

      int setGridKitComponentID(IdxT) override final;
      int verify() const override final;
      int allocate() override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      __attribute__((always_inline)) int evaluateInternalResidual(
          ScalarT* y, ScalarT* yp, ScalarT* wb, ScalarT* ws, ScalarT* wsp, ScalarT* f);

    private:
      void        setData(const ModelDataT& data);
      void        readInput();
      std::size_t yOutOffset() const;

      SignalsT signals_{1};

      std::vector<ScalarT> ws_;
      std::vector<ScalarT> wsp_;
      std::vector<IdxT>    ws_indices_;

      MatrixT D_{};
      MatrixT E_{};

      std::size_t          pole_count_{0};
      std::vector<PoleT>   poles_{};
      std::vector<MatrixT> A_{};
      std::vector<MatrixT> B_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitImpl.hpp>
#ifdef GRIDKIT_ENABLE_ENZYME
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitEnzyme.hpp>
#endif
