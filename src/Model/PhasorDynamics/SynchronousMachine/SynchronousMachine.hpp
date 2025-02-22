/**
 * @file SynchronousMachine.hpp
 * @author Slaven Peles (peless@ornl.gov)
 * @brief Declaration of a phasor dynamics branch model.
 * 
 * The model uses Cartesian coordinates.
 * 
 */
#pragma once

#include <Model/PhasorDynamics/Component.hpp>

// Forward declarations.
namespace GridKit
{
namespace PhasorDynamics
{
    template <class ScalarT, typename IdxT> class BusBase;
}
}

namespace GridKit
{
namespace PhasorDynamics
{
    /**
     * @brief Implementation of a pi-model branch between two buses.
     * 
     * The model is implemented in Cartesian coordinates. Positive current
     * direction is into the busses.
     *
     */
    template  <class ScalarT, typename IdxT>
    class SynchronousMachine : public Component<ScalarT, IdxT>
    {
        using Component<ScalarT, IdxT>::size_;
        using Component<ScalarT, IdxT>::nnz_;
        using Component<ScalarT, IdxT>::time_;
        using Component<ScalarT, IdxT>::alpha_;
        using Component<ScalarT, IdxT>::y_;
        using Component<ScalarT, IdxT>::yp_;
        using Component<ScalarT, IdxT>::tag_;
        using Component<ScalarT, IdxT>::f_;
        using Component<ScalarT, IdxT>::g_;
        using Component<ScalarT, IdxT>::yB_;
        using Component<ScalarT, IdxT>::ypB_;
        using Component<ScalarT, IdxT>::fB_;
        using Component<ScalarT, IdxT>::gB_;
        using Component<ScalarT, IdxT>::param_;

        using bus_type  = BusBase<ScalarT, IdxT>;
        using real_type = typename Component<ScalarT, IdxT>::real_type;

    public:
        SynchronousMachine(bus_type* bus);
        virtual ~SynchronousMachine();

        virtual int allocate() override;
        virtual int initialize() override;
        virtual int tagDifferentiable() override;
        virtual int evaluateResidual() override;
        virtual int evaluateJacobian() override;
        virtual int evaluateIntegrand() override;

        virtual int initializeAdjoint() override;
        virtual int evaluateAdjointResidual() override;
        // virtual int evaluateAdjointJacobian() override;
        virtual int evaluateAdjointIntegrand() override;

        virtual void updateTime(real_type /* t */, real_type /* a */) override
        {
        }


    private:
        ScalarT& Vr()
        {
            return bus_->Vr();
        }

        ScalarT& Vi()
        {
            return bus_->Vi();
        }

        ScalarT& Ir()
        {
            return bus_->Ir();
        }

        ScalarT& Ii()
        {
            return bus_->Ii();
        }

    private:
        void setParameters();

    private:
        bus_type* bus_{nullptr};

        // Generator parameters
        real_type omega0_{2*M_PI*60};
        real_type H_{3.0};
        real_type D_{0.0};
        real_type Ra_{0.0};
        real_type Tdop_{7.0};
        real_type Tdopp_{0.04};
        real_type Tqopp_{0.05};
        real_type Tqop_{0.75};
        real_type Xd_{2.1};
        real_type Xdp_{0.2};
        real_type Xdpp_{0.18};
        real_type Xq_{0.5};
        real_type Xqp_{0.5};
        real_type Xqpp_{0.18};
        real_type Xl_{0.15};
        real_type S10_{0.0};
        real_type S12_{0.0};

        // Simplifying constants calculated from parameters
        real_type SA_{0.0};
        real_type SB_{0.0};
        real_type Xd1_{0.0};
        real_type Xd2_{0.0};
        real_type Xd3_{0.0};
        real_type Xd4_{0.0};
        real_type Xd5_{0.0};
        real_type Xq1_{0.0};
        real_type Xq2_{0.0};
        real_type Xq3_{0.0};
        real_type Xq4_{0.0};
        real_type Xq5_{0.0};
        real_type Xqd_{0.0};
        real_type gg_{0.0};
        real_type bb_{0.0};
    
    };

} // namespace PhasorDynamics
} // namespace GridKit
