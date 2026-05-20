#pragma once

#include <complex>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/EMT/GridElement.hpp>
#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    class Bus final : public GridElement<ScalarT, IdxT>
    {
      using GridElement<ScalarT, IdxT>::size_;
      using GridElement<ScalarT, IdxT>::y_;
      using GridElement<ScalarT, IdxT>::yp_;
      using GridElement<ScalarT, IdxT>::tag_;
      using GridElement<ScalarT, IdxT>::f_;
      using GridElement<ScalarT, IdxT>::variable_indices_;
      using GridElement<ScalarT, IdxT>::residual_indices_;

    public:
      using RealT = typename GridElement<ScalarT, IdxT>::RealT;
      using DataT = BusData<RealT, IdxT>;

      static constexpr IdxT variable_count = 3;

      Bus();
      explicit Bus(const DataT& data);
      Bus(RealT vm, RealT va);
      virtual ~Bus();

      int                              setBusID(IdxT id);
      IdxT                             busID() const;
      const std::string&               name() const;
      const DataT&                     data() const;
      RealT                            frequency() const;
      RealT                            omega() const;
      PhaseVector<std::complex<RealT>> initialVoltagePhasor() const;

      int                               allocate() override final;
      int                               initialize() override final;
      int                               tagDifferentiable() override final;
      int                               evaluateResidual() override final;
      int                               evaluateJacobian() override final;
      int                               verify() const override final;
      bool                              hasJacobian() override final;
      void                              updateTime(RealT, RealT) override final;
      const Model::VariableMonitorBase* getMonitor() const override final;

      void addIntrinsicResidual(const std::vector<ScalarT>& y,
                                const std::vector<ScalarT>& yp,
                                std::vector<ScalarT>&       f) const;

      template <class LayoutT>
      void addIntrinsicJacobian(const LayoutT&, const std::vector<ScalarT>&, const std::vector<ScalarT>&, RealT, std::vector<RealT>&) const
      {
      }

      ScalarT&       voltage(IdxT phase);
      const ScalarT& voltage(IdxT phase) const;
      ScalarT&       residualCurrent(IdxT phase);
      const ScalarT& residualCurrent(IdxT phase) const;

    private:
      DataT data_{};
    };
  } // namespace EMT
} // namespace GridKit

#include <GridKit/Model/EMT/Bus/BusImpl.hpp>
