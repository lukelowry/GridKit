#pragma once

#include <complex>
#include <vector>

#include <GridKit/Model/EMT/InitialStateLayout.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentSignals.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VectorFitInternalVariables : size_t
    {
      out_a,
      out_b,
      out_c,
      MAXIMUM
    };

    enum class VectorFitExternalVariables : size_t
    {
      input_a,
      input_b,
      input_c,
      MAXIMUM
    };

    template <typename scalar_type, typename index_type>
    class VectorFit final
      : public PhasorDynamics::Component<scalar_type, index_type>,
        public InitialStateLayout
    {
      using ComponentT = PhasorDynamics::Component<scalar_type, index_type>;

      using ComponentT::abs_tol_;
      using ComponentT::allocated_;
      using ComponentT::alpha_;
      using ComponentT::coo_jac_;
      using ComponentT::f_;
      using ComponentT::gridkit_component_id_;
      using ComponentT::J_cols_buffer_;
      using ComponentT::J_rows_buffer_;
      using ComponentT::J_vals_buffer_;
      using ComponentT::nnz_;
      using ComponentT::residual_indices_;
      using ComponentT::size_;
      using ComponentT::tag_;
      using ComponentT::variable_indices_;
      using ComponentT::y_;
      using ComponentT::yp_;

    public:
      using ScalarT    = scalar_type;
      using IdxT       = index_type;
      using RealT      = typename ComponentT::RealT;
      using ModelDataT = VectorFitData<RealT, IdxT>;
      using SignalsT   = PhasorDynamics::ComponentSignals<ScalarT,
                                                          IdxT,
                                                          VectorFitInternalVariables,
                                                          VectorFitExternalVariables>;

      explicit VectorFit(const ModelDataT& data);
      ~VectorFit();

      int setGridKitComponentID(IdxT) override final;
      int allocate() override final;
      int verify() const override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      void appendInitialStateVariables(
          std::vector<InitialStateVariable>&) const override;

      SignalsT& getSignals()
      {
        return signals_;
      }

      __attribute__((always_inline)) inline int evaluateInternalResidual(
          const ScalarT* y,
          const ScalarT* yp,
          const ScalarT* ws,
          const ScalarT* wsp,
          ScalarT*       f);

    private:
      enum class PoleKind
      {
        Real,
        ComplexFirst,
        ComplexSecond
      };

      void initializeParameters(const ModelDataT& data);
      bool matrixIsFinite(const ABCMatrix<RealT>& matrix) const;
      bool complexMatrixIsFinite(const ABCMatrix<std::complex<RealT>>& matrix) const;
      bool complexMatrixIsReal(const ABCMatrix<std::complex<RealT>>& matrix) const;
      bool complexMatricesAreConjugate(
          const ABCMatrix<std::complex<RealT>>& lhs,
          const ABCMatrix<std::complex<RealT>>& rhs) const;
      bool matrixIsZero(const ABCMatrix<RealT>& matrix) const;
      bool approximatelyEqual(RealT lhs, RealT rhs) const;
      bool approximatelyConjugate(const std::complex<RealT>& lhs,
                                  const std::complex<RealT>& rhs) const;
      void appendJacobianEntry(IdxT row, IdxT column, RealT value);

      ABCMatrix<RealT> D_{};
      ABCMatrix<RealT> E_{};

      std::vector<std::complex<RealT>>            poles_{};
      std::vector<ABCMatrix<std::complex<RealT>>> residues_{};
      std::vector<PoleKind>                       pole_kinds_{};

      bool has_D_{false};
      bool has_E_{false};
      bool has_poles_{false};
      bool has_residues_{false};

      IdxT pole_count_{0};
      IdxT output_offset_{0};

      SignalsT signals_{};

      std::vector<ScalarT> ws_{};
      std::vector<ScalarT> wsp_{};
      std::vector<IdxT>    ws_indices_{};
    };
  } // namespace EMT
} // namespace GridKit
