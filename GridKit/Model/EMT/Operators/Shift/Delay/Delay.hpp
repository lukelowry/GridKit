#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <GridKit/LinearAlgebra/SparseMatrix/COO_Matrix.hpp>
#include <GridKit/LinearAlgebra/SparseMatrix/CsrMatrix.hpp>
#include <GridKit/Model/Evaluator.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class Delay : public Model::Evaluator<ScalarT, IdxT>
    {
    public:
      using Base                 = Model::Evaluator<ScalarT, IdxT>;
      using RealT                = typename Base::RealT;
      using MatrixT              = typename Base::MatrixT;
      using CsrMatrixT           = typename Base::CsrMatrixT;
      using InputFunction        = std::function<ScalarT(RealT)>;
      using InitialStateFunction = std::function<void(IdxT, RealT, RealT, ScalarT&, ScalarT&)>;

      Delay(RealT tau, RealT fmax, InputFunction input, InputFunction input_derivative, InitialStateFunction initial_state = {});
      ~Delay() override = default;

      int allocate() override;
      int initialize() override;
      int tagDifferentiable() override;
      int setAbsoluteTolerance(RealT rel_tol) override;
      int evaluateResidual() override;
      int evaluateJacobian() override;
      int evaluateIntegrand() override;

      int initializeAdjoint() override;
      int evaluateAdjointResidual() override;
      int evaluateAdjointIntegrand() override;

      IdxT size() override;
      IdxT nnz() override;
      bool hasJacobian() override;
      IdxT sizeQuadrature() override;
      IdxT sizeParams() override;
      void updateTime(RealT t, RealT a) override;

      CsrMatrixT* getCsrJacobian() const override;

      std::vector<ScalarT>&       absoluteTolerance() override;
      const std::vector<ScalarT>& absoluteTolerance() const override;
      std::vector<ScalarT>&       y() override;
      const std::vector<ScalarT>& y() const override;
      std::vector<ScalarT>&       yp() override;
      const std::vector<ScalarT>& yp() const override;
      std::vector<bool>&          tag() override;
      const std::vector<bool>&    tag() const override;
      std::vector<ScalarT>&       yB() override;
      const std::vector<ScalarT>& yB() const override;
      std::vector<ScalarT>&       ypB() override;
      const std::vector<ScalarT>& ypB() const override;
      std::vector<ScalarT>&       param() override;
      const std::vector<ScalarT>& param() const override;
      std::vector<ScalarT>&       param_up() override;
      const std::vector<ScalarT>& param_up() const override;
      std::vector<ScalarT>&       param_lo() override;
      const std::vector<ScalarT>& param_lo() const override;
      std::vector<ScalarT>&       getResidual() override;
      const std::vector<ScalarT>& getResidual() const override;
      MatrixT&                    getJacobian() override;
      const MatrixT&              getJacobian() const override;
      std::vector<ScalarT>&       getIntegrand() override;
      const std::vector<ScalarT>& getIntegrand() const override;
      std::vector<ScalarT>&       getAdjointResidual() override;
      const std::vector<ScalarT>& getAdjointResidual() const override;
      std::vector<ScalarT>&       getAdjointIntegrand() override;
      const std::vector<ScalarT>& getAdjointIntegrand() const override;

      IdxT    sectionCount() const;
      RealT   sectionTime() const;
      ScalarT input() const;
      ScalarT output() const;

    private:
      static IdxT computeSectionCount(RealT tau, RealT fmax);

      void initializeStorage();
      void initializeJacobian();

      RealT tau_;
      RealT fmax_;
      IdxT  n_;
      RealT T_;

      InputFunction        input_function_;
      InputFunction        input_derivative_;
      InitialStateFunction initial_state_function_;

      RealT   time_{};
      RealT   alpha_{};
      ScalarT input_value_{};

      std::vector<ScalarT> y_;
      std::vector<ScalarT> yp_;
      std::vector<bool>    tag_;
      std::vector<ScalarT> abs_tol_;
      std::vector<ScalarT> f_;
      std::vector<ScalarT> g_;
      std::vector<ScalarT> yB_;
      std::vector<ScalarT> ypB_;
      std::vector<ScalarT> fB_;
      std::vector<ScalarT> gB_;
      std::vector<ScalarT> param_;
      std::vector<ScalarT> param_up_;
      std::vector<ScalarT> param_lo_;

      MatrixT                     jac_;
      std::unique_ptr<CsrMatrixT> csr_jac_;
    };
  } // namespace EMT
} // namespace GridKit
