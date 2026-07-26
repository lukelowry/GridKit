#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

#include <GridKit/Definitions.hpp>
#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit::Testing::EMTTest
{
  using RealT   = double;
  using IdxT    = std::size_t;
  using SystemT = EMT::SystemModel<RealT, IdxT>;
  using DataT   = EMT::SystemModelData<RealT, IdxT>;

  inline DataT loadFixtureData()
  {
    return EMT::parseSystemModelData(std::filesystem::path{EMT_TEST_FIXTURE});
  }

  /// Read the constant and linear coefficients of a named submodel block.
  template <typename ComponentDataT, typename SubmodelT>
  EMT::RationalCoefficients<RealT> rationalBlock(const ComponentDataT& data,
                                                 SubmodelT             submodel)
  {
    return EMT::rationalCoefficients(data.submodels.at(submodel));
  }

  /// Overwrite the constant and linear coefficients of a named submodel block.
  template <typename ComponentDataT, typename SubmodelT>
  void setRationalBlock(ComponentDataT&              data,
                        SubmodelT                    submodel,
                        const EMT::ABCMatrix<RealT>& D,
                        const EMT::ABCMatrix<RealT>& E)
  {
    auto& block                                   = data.submodels[submodel];
    block.parameters[EMT::VectorFitParameters::D] = D;
    block.parameters[EMT::VectorFitParameters::E] = E;
  }

  inline bool isThreeBusMutuallyCoupled(const DataT& data)
  {
    // Three feeder buses plus the terminal bus behind the fault switch.
    if (data.bus.size() != 4 || data.line_lumped.size() != 2)
    {
      return false;
    }

    for (const auto& line : data.line_lumped)
    {
      const auto  series = rationalBlock(line, EMT::LineLumpedSubmodels::Zp);
      const auto  shunt  = rationalBlock(line, EMT::LineLumpedSubmodels::Yp);
      const auto& Rp     = series.D;
      const auto& Lp     = series.E;
      const auto& Cp     = shunt.E;

      if (Rp[0][1] == 0.0 || Rp[0][2] == 0.0 || Rp[1][2] == 0.0
          || Lp[0][1] == 0.0 || Lp[0][2] == 0.0 || Lp[1][2] == 0.0
          || Cp[0][1] == 0.0 || Cp[0][2] == 0.0 || Cp[1][2] == 0.0)
      {
        return false;
      }
    }
    return true;
  }

  inline std::size_t componentCount(const DataT& data)
  {
    return data.constant_source.size()
           + data.voltage_source.size()
           + data.line_lumped.size()
           + data.loadz.size()
           + data.switches.size()
           + data.vector_fit.size();
  }

  inline std::unique_ptr<SystemT> makeFixtureSystem(const DataT& data)
  {
    auto system = std::make_unique<SystemT>(data);
    system->updateTime(0.0, 1.0);
    system->allocate();
    system->tagDifferentiable();
    system->setAbsoluteTolerance(1.0e-8);
    system->initialize();
    return system;
  }

  template <typename ComponentT>
  ComponentT* findComponent(SystemT&     system,
                            const DataT& data,
                            std::size_t  occurrence = 0)
  {
    std::size_t found = 0;
    for (std::size_t id = 0; id < componentCount(data); ++id)
    {
      auto* component = dynamic_cast<ComponentT*>(system.getComponent(id));
      if (component != nullptr)
      {
        if (found == occurrence)
        {
          return component;
        }
        ++found;
      }
    }
    return nullptr;
  }

  template <typename ComponentT>
  RealT cooValue(ComponentT& component, IdxT row, IdxT column)
  {
    RealT result   = 0.0;
    auto* jacobian = component.getCooJacobian();
    if (jacobian == nullptr)
    {
      return result;
    }
    for (IdxT entry = 0; entry < jacobian->getNnz(); ++entry)
    {
      if (jacobian->getRowData()[entry] == row
          && jacobian->getColData()[entry] == column)
      {
        result += jacobian->getValues()[entry];
      }
    }
    return result;
  }

  template <typename ComponentT>
  std::vector<std::pair<IdxT, IdxT>> cooPattern(ComponentT& component)
  {
    std::vector<std::pair<IdxT, IdxT>> result;
    auto*                              jacobian = component.getCooJacobian();
    if (jacobian == nullptr)
    {
      return result;
    }
    result.reserve(jacobian->getNnz());
    for (IdxT entry = 0; entry < jacobian->getNnz(); ++entry)
    {
      result.emplace_back(jacobian->getRowData()[entry],
                          jacobian->getColData()[entry]);
    }
    return result;
  }
} // namespace GridKit::Testing::EMTTest
