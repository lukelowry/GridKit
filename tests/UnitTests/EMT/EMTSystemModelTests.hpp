#pragma once

#include <GridKit/LinearAlgebra/MemoryUtils.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "EMTTestHelpers.hpp"

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTSystemModelTests
    {
    private:
      using RealT = typename EMT::Component<ScalarT, IdxT>::RealT;

    public:
      TestOutcome twoBusSystemResidualJacobianAndTags()
      {
        TestStatus success = true;

        auto                            data = makeTwoBusSystemData<RealT, IdxT>();
        EMT::SystemModel<ScalarT, IdxT> system(data);
        success *= (system.allocate() == 0);
        success *= (system.size() == 12);
        success *= (system.nnz() > 0);
        success *= (system.getCsrJacobian() != nullptr);

        success *= (system.initialize() == 0);
        system.updateTime(0.013, 4.0);
        success *= (system.tagDifferentiable() == 0);
        for (std::size_t idx = 0; idx < static_cast<std::size_t>(system.size()); ++idx)
        {
          success *= system.tag()[idx];
        }

        success      *= (system.evaluateResidual() == 0);
        auto* branch  = dynamic_cast<EMT::BranchLumpedConstant<ScalarT, IdxT>*>(system.getComponent(0));
        auto* load    = dynamic_cast<EMT::LoadRL<ScalarT, IdxT>*>(system.getComponent(1));
        auto* source  = dynamic_cast<EMT::VoltageSource<ScalarT, IdxT>*>(system.getComponent(2));
        success      *= (branch != nullptr);
        success      *= (load != nullptr);
        success      *= (source != nullptr);

        std::vector<double> branch_y{
            system.y()[6], system.y()[7], system.y()[8], system.y()[0], system.y()[1], system.y()[2], system.y()[3], system.y()[4], system.y()[5]};
        std::vector<double> branch_yp{
            system.yp()[6], system.yp()[7], system.yp()[8], system.yp()[0], system.yp()[1], system.yp()[2], system.yp()[3], system.yp()[4], system.yp()[5]};
        std::vector<double> branch_f(9, 0.0);
        branch->residual(branch_y.data(), branch_yp.data(), branch_f.data(), 0.013);

        std::vector<double> load_y{
            system.y()[9], system.y()[10], system.y()[11], system.y()[3], system.y()[4], system.y()[5]};
        std::vector<double> load_yp{
            system.yp()[9], system.yp()[10], system.yp()[11], system.yp()[3], system.yp()[4], system.yp()[5]};
        std::vector<double> load_f(6, 0.0);
        load->residual(load_y.data(), load_yp.data(), load_f.data(), 0.013);

        std::vector<double> source_y{system.y()[0], system.y()[1], system.y()[2]};
        std::vector<double> source_yp(3, 0.0);
        std::vector<double> source_f(3, 0.0);
        source->residual(source_y.data(), source_yp.data(), source_f.data(), 0.013);

        success *= isEqual(system.getResidual()[0], source_f[0] + branch_f[3], 1.0e-10);
        success *= isEqual(system.getResidual()[3], branch_f[6] + load_f[3], 1.0e-10);

        success *= (system.evaluateJacobian() == 0);
        success *= csrMatchesFiniteDifference(system, 4.0);

        return success.report(__func__);
      }

    private:
      bool csrMatchesFiniteDifference(EMT::SystemModel<ScalarT, IdxT>& system, RealT alpha)
      {
        auto*       csr     = system.getCsrJacobian();
        const auto* row_ptr = csr->getRowData(LinearAlgebra::memory::HOST);
        const auto* columns = csr->getColData(LinearAlgebra::memory::HOST);
        const auto* values  = csr->getValues(LinearAlgebra::memory::HOST);

        system.evaluateResidual();
        const auto      base_f  = system.getResidual();
        const auto      base_y  = system.y();
        const auto      base_yp = system.yp();
        constexpr RealT eps     = 1.0e-7;

        for (IdxT row = 0; row < system.size(); ++row)
        {
          for (IdxT slot = row_ptr[row]; slot < row_ptr[row + 1]; ++slot)
          {
            const auto col                              = columns[slot];
            system.y()                                  = base_y;
            system.yp()                                 = base_yp;
            system.y()[static_cast<std::size_t>(col)]  += eps;
            system.yp()[static_cast<std::size_t>(col)] += alpha * eps;
            system.evaluateResidual();
            const RealT fd =
                (system.getResidual()[static_cast<std::size_t>(row)]
                 - base_f[static_cast<std::size_t>(row)])
                / eps;
            if (!isEqual(values[slot], fd, 1.0e-4))
            {
              system.y()  = base_y;
              system.yp() = base_yp;
              return false;
            }
          }
        }

        system.y()  = base_y;
        system.yp() = base_yp;
        system.evaluateResidual();
        return true;
      }
    };
  } // namespace Testing
} // namespace GridKit
