#include <memory>
#include <stdexcept>
#include <vector>

#include <GridKit/Model/EMT/BusVoltageContribution.hpp>
#include <GridKit/Model/PhasorDynamics/Component.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;

  class BusVoltageTestComponent final
    : public PhasorDynamics::Component<RealT, IdxT>,
      public EMT::BusVoltageContributor<RealT, IdxT>
  {
    using Base          = PhasorDynamics::Component<RealT, IdxT>;
    using ContributionT = EMT::BusVoltageContribution<RealT, IdxT>;
    using Base::allocated_;
    using Base::f_;
    using Base::gridkit_component_id_;
    using Base::residual_indices_;
    using Base::size_;
    using Base::tag_;
    using Base::variable_indices_;

  public:
    BusVoltageTestComponent(
        IdxT                         bus_id,
        const EMT::ABCMatrix<RealT>& derivative_block,
        const EMT::ABCMatrix<RealT>& algebraic_block = {})
      : contribution_{bus_id, derivative_block, algebraic_block}
    {
      size_ = 3;
    }

    int setGridKitComponentID(IdxT component_id) override
    {
      gridkit_component_id_ = component_id;
      return 0;
    }

    int allocate() override
    {
      if (!allocated_)
      {
        this->allocateVectors(size_);
      }
      tag_.assign(3, false);
      variable_indices_.resize(3);
      residual_indices_.resize(3);
      for (IdxT phase = 0; phase < 3; ++phase)
      {
        this->setVariableIndex(phase, phase);
        this->setResidualIndex(phase, phase);
      }
      allocated_ = true;
      return 0;
    }

    int verify() const override
    {
      return 0;
    }

    int initialize() override
    {
      ++initialize_calls_;
      return 0;
    }

    int tagDifferentiable() override
    {
      tag_.assign(3, false);
      return 0;
    }

    int setAbsoluteTolerance(RealT) override
    {
      return 0;
    }

    int evaluateResidual() override
    {
      f_.setToConst(RealT{0.0});
      return 0;
    }

    int evaluateJacobian() override
    {
      return 0;
    }

    void appendBusVoltageContributions(
        std::vector<ContributionT>& contributions) const override
    {
      contributions.push_back(contribution_);
    }

    std::size_t initializeCalls() const
    {
      return initialize_calls_;
    }

  private:
    ContributionT contribution_;
    std::size_t   initialize_calls_{0};
  };

  bool classificationMatches(
      const std::vector<EMT::ABCMatrix<RealT>>& blocks,
      EMT::BusVoltageClass                      expected)
  {
    EMT::BusData<RealT, IdxT> data;
    data.name   = "classification";
    data.bus_id = 1;

    EMT::Bus<RealT, IdxT>                                 bus(data);
    std::vector<std::unique_ptr<BusVoltageTestComponent>> components;
    SystemT                                               system;
    system.addBus(&bus);

    components.reserve(blocks.size());
    for (const auto& block : blocks)
    {
      components.push_back(
          std::make_unique<BusVoltageTestComponent>(data.bus_id, block));
      system.addComponent(components.back().get());
    }

    if (system.allocate() != 0 || system.tagDifferentiable() != 0)
    {
      return false;
    }

    bool result = bus.voltageClass() == expected;
    for (const auto& component : components)
    {
      result = result && component->initializeCalls() == 0;
    }

    const bool differential = expected == EMT::BusVoltageClass::differential;
    for (IdxT phase = 0; phase < 3; ++phase)
    {
      result = result && bus.tag()[phase] == differential;
      result = result && system.tag()[phase] == differential;
    }
    return result;
  }

  bool classificationRejected(
      const std::vector<EMT::ABCMatrix<RealT>>& blocks)
  {
    EMT::BusData<RealT, IdxT> data;
    data.name   = "classification";
    data.bus_id = 1;

    EMT::Bus<RealT, IdxT>                                 bus(data);
    std::vector<std::unique_ptr<BusVoltageTestComponent>> components;
    SystemT                                               system;
    system.addBus(&bus);

    components.reserve(blocks.size());
    for (const auto& block : blocks)
    {
      components.push_back(
          std::make_unique<BusVoltageTestComponent>(data.bus_id, block));
      system.addComponent(components.back().get());
    }

    const bool rejected = throws<std::runtime_error>([&system]
                                                     { system.allocate(); });
    bool       result   = rejected;
    for (const auto& component : components)
    {
      result = result && component->initializeCalls() == 0;
    }
    return result;
  }

  bool singleContributionClassificationMatches(
      const EMT::ABCMatrix<RealT>& derivative_block,
      const EMT::ABCMatrix<RealT>& algebraic_block,
      EMT::BusVoltageClass         expected,
      bool                         differentiated_kcl)
  {
    EMT::BusData<RealT, IdxT> data;
    data.name   = "classification";
    data.bus_id = 1;

    EMT::Bus<RealT, IdxT>   bus(data);
    BusVoltageTestComponent component(
        data.bus_id, derivative_block, algebraic_block);
    SystemT system;
    system.addBus(&bus);
    system.addComponent(&component);

    if (system.allocate() != 0 || system.tagDifferentiable() != 0)
    {
      return false;
    }
    return bus.voltageClass() == expected
           && bus.differentiatedKCL() == differentiated_kcl;
  }

  bool singleContributionClassificationRejected(
      const EMT::ABCMatrix<RealT>& derivative_block,
      const EMT::ABCMatrix<RealT>& algebraic_block)
  {
    EMT::BusData<RealT, IdxT> data;
    data.name   = "classification";
    data.bus_id = 1;

    EMT::Bus<RealT, IdxT>   bus(data);
    BusVoltageTestComponent component(
        data.bus_id, derivative_block, algebraic_block);
    SystemT system;
    system.addBus(&bus);
    system.addComponent(&component);
    return throws<std::runtime_error>([&system]
                                      { system.allocate(); });
  }

  TestOutcome abcInitializationAndKclReset()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    for (const auto& bus_data : data.bus)
    {
      auto* bus  = system->getBus(bus_data.bus_id);
      success   *= bus->size() == 3;
      success   *= bus->busID() == bus_data.bus_id;
      success   *= bus->Va() == 0.0;
      success   *= bus->Vb() == 0.0;
      success   *= bus->Vc() == 0.0;
      success   *= bus->Vap() == 0.0;
      success   *= bus->Vbp() == 0.0;
      success   *= bus->Vcp() == 0.0;

      // The terminal bus behind the fault switch carries only algebraic
      // branches, so its voltage is algebraic.
      const bool differential  = bus_data.bus_id != 6321;
      success                 *= bus->voltageClass()
                 == (differential ? EMT::BusVoltageClass::differential
                                  : EMT::BusVoltageClass::algebraic);
      success *= bus->tag()[0] == differential
                 && bus->tag()[1] == differential
                 && bus->tag()[2] == differential;

      bus->evaluateResidual();
      bus->Ia() += 1.25;
      bus->Ib() += -2.50;
      bus->Ic() += 3.75;
      bus->Ia() += -0.50;
      bus->Ib() += 1.00;
      bus->Ic() += -1.50;
      bus->Ia() += 0.125;
      bus->Ib() += 0.250;
      bus->Ic() += 0.375;
      success   *= isEqual(bus->Ia(), 0.875, 1.0e-15);
      success   *= isEqual(bus->Ib(), -1.250, 1.0e-15);
      success   *= isEqual(bus->Ic(), 2.625, 1.0e-15);

      success *= bus->evaluateResidual() == 0;
      success *= bus->Ia() == 0.0;
      success *= bus->Ib() == 0.0;
      success *= bus->Ic() == 0.0;
    }

    return success.report(__func__);
  }

  TestOutcome structuralVoltageClassification()
  {
    TestStatus success = true;

    const EMT::ABCMatrix<RealT> zero{};
    const EMT::ABCMatrix<RealT> full{{{1.0, 0.0, 0.0},
                                      {0.0, 1.0, 0.0},
                                      {0.0, 0.0, 1.0}}};
    const EMT::ABCMatrix<RealT> rank_one{{{1.0, 0.0, 0.0},
                                          {0.0, 0.0, 0.0},
                                          {0.0, 0.0, 0.0}}};
    const EMT::ABCMatrix<RealT> rank_two{{{1.0, 0.0, 0.0},
                                          {0.0, 1.0, 0.0},
                                          {0.0, 0.0, 0.0}}};
    const EMT::ABCMatrix<RealT> large_rank_one{{{1.0e200, 0.0, 0.0},
                                                {0.0, 0.0, 0.0},
                                                {0.0, 0.0, 0.0}}};
    const EMT::ABCMatrix<RealT> small_rank_two{{{0.0, 0.0, 0.0},
                                                {0.0, 1.0e-200, 0.0},
                                                {0.0, 0.0, 1.0e-200}}};

    success *= classificationRejected({});
    success *= classificationMatches(
        {zero}, EMT::BusVoltageClass::algebraic);
    success *= singleContributionClassificationMatches(
        zero, zero, EMT::BusVoltageClass::algebraic, true);
    success *= singleContributionClassificationMatches(
        zero, full, EMT::BusVoltageClass::algebraic, false);
    success *= singleContributionClassificationRejected(zero, rank_one);
    success *= singleContributionClassificationRejected(zero, rank_two);
    success *= classificationMatches(
        {full}, EMT::BusVoltageClass::differential);
    success *= singleContributionClassificationMatches(
        full, zero, EMT::BusVoltageClass::differential, false);
    success *= classificationRejected({rank_one});
    success *= classificationRejected({rank_two});
    success *= classificationMatches(
        {large_rank_one, small_rank_two},
        EMT::BusVoltageClass::differential);

    return success.report(__func__);
  }

  TestOutcome fixedJacobianStructure()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    for (const auto& bus_data : data.bus)
    {
      auto* bus       = system->getBus(bus_data.bus_id);
      success        *= bus->evaluateJacobian() == 0;
      auto* jacobian  = bus->getCooJacobian();
      success        *= jacobian != nullptr;
      if (jacobian != nullptr)
      {
        success *= jacobian->getNnz() == 9;
        for (std::size_t entry = 0; entry < jacobian->getNnz(); ++entry)
        {
          success *= jacobian->getValues()[entry] == 0.0;
        }
      }

      success *= componentJacobianMatchesReferences(
          *system, *bus, data, {3.0, 37.0}, false);
    }

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += abcInitializationAndKclReset();
  result += structuralVoltageClassification();
  result += fixedJacobianStructure();
  return result.summary();
}
