#pragma once

#include <cstddef>
#include <vector>

#include <GridKit/Constants.hpp>
#include <GridKit/Model/PhasorDynamics/BusBase.hpp>
#include <GridKit/Utilities/Errors.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    class Bus final : public PhasorDynamics::BusBase<scalar_type, index_type>
    {
      using PhasorDynamics::BusBase<scalar_type, index_type>::size_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::nnz_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::bus_id_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::y_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::yp_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::f_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::tag_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::abs_tol_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::J_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::variable_indices_;
      using PhasorDynamics::BusBase<scalar_type, index_type>::residual_indices_;

    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using BaseT   = PhasorDynamics::BusBase<ScalarT, IdxT>;
      using RealT   = typename BaseT::RealT;

      Bus();
      explicit Bus(std::vector<RealT> v0);
      Bus(IdxT bus_id, std::vector<RealT> v0);
      ~Bus() override;

      int verify() const override final;
      int setBusID(IdxT bus_id) override final;
      int allocate() override final;
      int initialize() override final;
      int tagDifferentiable() override final;
      int setAbsoluteTolerance(RealT rel_tol) override final;
      int evaluateResidual() override final;
      int evaluateJacobian() override final;

      IdxT phaseCount() const
      {
        return size_;
      }

      ScalarT& V(IdxT n)
      {
        return y_[static_cast<std::size_t>(n)];
      }

      const ScalarT& V(IdxT n) const
      {
        return y_[static_cast<std::size_t>(n)];
      }

      ScalarT& Vp(IdxT n)
      {
        return yp_[static_cast<std::size_t>(n)];
      }

      const ScalarT& Vp(IdxT n) const
      {
        return yp_[static_cast<std::size_t>(n)];
      }

      ScalarT& I(IdxT n)
      {
        return f_[static_cast<std::size_t>(n)];
      }

      const ScalarT& I(IdxT n) const
      {
        return f_[static_cast<std::size_t>(n)];
      }

      [[noreturn]] ScalarT& Vr() override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] const ScalarT& Vr() const override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] ScalarT& Vi() override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] const ScalarT& Vi() const override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] ScalarT& Ir() override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] const ScalarT& Ir() const override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] ScalarT& Ii() override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

      [[noreturn]] const ScalarT& Ii() const override final
      {
        throw GridKit::Utilities::NotImplementedError(__func__);
      }

    private:
      std::vector<RealT> v0_;
    };
  } // namespace EMT
} // namespace GridKit
