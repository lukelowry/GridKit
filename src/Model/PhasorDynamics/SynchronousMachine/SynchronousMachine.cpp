/**
 * @file SynchronousMachine.hpp
 * @author Slaven Peles (peless@ornl.gov)
 * @brief Definition of a phasor dynamics branch model.
 * 
 * The model uses Cartesian coordinates.
 * 
 */

#include <iostream>
#include <cmath>
#include <Model/PhasorDynamics/Bus/Bus.hpp>
#include <PowerSystemData.hpp>

#include "SynchronousMachine.hpp"

namespace GridKit
{
namespace PhasorDynamics
{
    /*!
    * @brief Constructor for a pi-model branch
    *
    * Arguments passed to ModelEvaluatorImpl:
    * - Number of equations = 0
    * - Number of independent variables = 0
    * - Number of quadratures = 0
    * - Number of optimization parameters = 0
    */
    template <class ScalarT, typename IdxT>
    SynchronousMachine<ScalarT, IdxT>::SynchronousMachine(bus_type* bus)
    : bus_(bus)
    {
        size_ = 0;
    }

    /**
     * @brief Destroy the SynchronousMachine
     * 
     * @tparam ScalarT 
     * @tparam IdxT 
     */
    template <class ScalarT, typename IdxT>
    SynchronousMachine<ScalarT, IdxT>::~SynchronousMachine()
    {
        //std::cout << "Destroy SynchronousMachine..." << std::endl;
    }

    /*!
    * @brief allocate method computes sparsity pattern of the Jacobian.
    */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::allocate()
    {
        //std::cout << "Allocate SynchronousMachine..." << std::endl;
        return 0;
    }

    /**
     * Initialization of the branch model
     *
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::initialize()
    {
        return 0;
    }

    /**
     * \brief Identify differential variables.
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::tagDifferentiable()
    {
        return 0;
    }

    /**
     * \brief Residual contribution of the branch is pushed to the
     * two terminal buses.
     * 
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::evaluateResidual()
    {
        /* Read variables */
        ScalarT delta = y_[0];
        ScalarT omega = y_[1];
        ScalarT Eqp = y_[2];
        ScalarT psidp = y_[3];
        ScalarT psiqp = y_[4];
        ScalarT Edp = y_[5];
        ScalarT psiqpp = y_[6]; 
        ScalarT psidpp = y_[7]; 
        ScalarT psipp = y_[8]; 
        ScalarT ksat = y_[9];
        ScalarT vd = y_[10];
        ScalarT vq = y_[11];
        ScalarT telec = y_[12];
        ScalarT id = y_[13];
        ScalarT iq = y_[14];
        ScalarT ir = y_[15];
        ScalarT ii = y_[16];
        ScalarT pmech = y_[17];
        ScalarT efd = y_[18];
        ScalarT inr = y_[19];
        ScalarT ini = y_[20];
        ScalarT vr = y_[21]; 
        ScalarT vi = y_[22]; 
        ScalarT vr_inf = y_[23];
        ScalarT vi_inf = y_[24];
    
        /* Read derivatives */
        ScalarT delta_dot = yp_[0];
        ScalarT omega_dot = yp_[1];
        ScalarT Eqp_dot = yp_[2];
        ScalarT psidp_dot = yp_[3];
        ScalarT psiqp_dot = yp_[4];
        ScalarT Edp_dot = yp_[5];

        /* 6 GENROU differential equations */
        f_[0] = delta_dot - omega*omega0_;
        f_[1] = omega_dot - (1/(2* H_ )) * ((pmech - D_ * omega) / (1 + omega) 
            - telec);
        f_[2] = Eqp_dot - (1 / Tdop_ ) * (efd - (Eqp + Xd1_ *(id + Xd3_ *(Eqp 
            - psidp - Xd2_ *id)) + psidpp*ksat ));
        f_[3] = psidp_dot - (1/ Tdopp_ ) * (Eqp - psidp - Xd2_ *id);
        f_[4] = psiqp_dot - (1/ Tqopp_ ) * (Edp - psiqp + Xq2_ *iq);
        f_[5] = Edp_dot - (1/Tqop_) * (-Edp + Xqd_ *psiqpp*ksat 
            + Xq1_ *(iq - Xq3_ *(Edp + iq* Xq2_ - psiqp)));
        
        /* 11 GENROU algebraic equations */
        f_[6] = psiqpp - (-psiqp * Xq4_ - Edp * Xq5_);
        f_[7] = psidpp - (psidp * Xd4_ + Eqp * Xd5_);
        f_[8] = psipp - sqrt(pow(psidpp, 2.0) + pow(psiqpp, 2.0));
        f_[9] = ksat - SB_*pow(psipp - SA_, 2.0);
        f_[10] = vd + psiqpp * (1 + omega);
        f_[11] = vq - psidpp * (1 + omega);
        f_[12] = telec - ((psidpp - id*Xdpp_)*iq - (psiqpp - iq*Xdpp_)*id);
        f_[13] = id - (ir*sin(delta) - ii*cos(delta));
        f_[14] = iq - (ir*cos(delta) + ii*sin(delta));
        f_[15] = ir + gg_*vr - bb_*vi - inr;
        f_[16] = ii + bb_*vr + gg_*vi - ini;
        
        /* 2 GENROU control inputs are set to constant for this example */
        // f_[17] = pmech - pmech_set_;
        // f_[18] = efd - efd_set_;

        /* 2 GENROU current source definitions */
        f_[19] = inr - (gg_*(sin(delta)*vd + cos(delta)*vq) 
            - bb_*(-cos(delta)*vd + sin(delta)*vq));
        f_[20] = ini - (bb_*(sin(delta)*vd + cos(delta)*vq) 
            + gg_*(-cos(delta)*vd + sin(delta)*vq));

        /* Bus 1 network constraints (current balance) */
        // g = gg_;
        // double bbr = - 1/branch_x_;
        // b = bb_ + bbr + fault_b_;
        // f_[21] = -inr + vr*g - vi*b + vi_inf*bbr;
        // f_[22] = -ini + vr*b + vi*g - vr_inf*bbr;

        /* Bus 2 network constraints (fixed infinite bus voltage) */
        // f_[23] = vr_inf - vr_inf_set_;
        // f_[24] = vi_inf - vi_inf_set_;

        return 0;
    }

    /**
     * @brief Jacobian evaluation not implemented yet
     * 
     * @tparam ScalarT - scalar data type
     * @tparam IdxT    - matrix index data type
     * @return int - error code, 0 = success
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::evaluateJacobian()
    {
        std::cout << "Evaluate Jacobian for SynchronousMachine..." << std::endl;
        std::cout << "Jacobian evaluation not implemented!" << std::endl;
        return 0;
    }

    /**
     * @brief Integrand (objective) evaluation not implemented yet
     * 
     * @tparam ScalarT - scalar data type
     * @tparam IdxT    - matrix index data type
     * @return int - error code, 0 = success
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::evaluateIntegrand()
    {
        // std::cout << "Evaluate Integrand for SynchronousMachine..." << std::endl;
        return 0;
    }

    /**
     * @brief Adjoint initialization not implemented yet
     * 
     * @tparam ScalarT - scalar data type
     * @tparam IdxT    - matrix index data type
     * @return int - error code, 0 = success
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::initializeAdjoint()
    {
        //std::cout << "Initialize adjoint for SynchronousMachine..." << std::endl;
        return 0;
    }

    /**
     * @brief Adjoint residual evaluation not implemented yet
     * 
     * @tparam ScalarT - scalar data type
     * @tparam IdxT    - matrix index data type
     * @return int - error code, 0 = success
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::evaluateAdjointResidual()
    {
        // std::cout << "Evaluate adjoint residual for SynchronousMachine..." << std::endl;
        return 0;
    }

    /**
     * @brief Adjoint integrand (objective) evaluation not implemented yet
     * 
     * @tparam ScalarT - scalar data type
     * @tparam IdxT    - matrix index data type
     * @return int - error code, 0 = success
     */
    template <class ScalarT, typename IdxT>
    int SynchronousMachine<ScalarT, IdxT>::evaluateAdjointIntegrand()
    {
        // std::cout << "Evaluate adjoint Integrand for SynchronousMachine..." << std::endl;
        return 0;
    }

    /**
     * @brief 
     * 
     */
    template <class ScalarT, typename IdxT>
    void SynchronousMachine<ScalarT, IdxT>::setParameters()
    {
        SA_ = 0;
        SB_ = 0;
        if (S12_ != 0)
        {
            real_type s112 = sqrt(S10_ / S12_);
            SA_ = (1.2*s112 + 1) / (s112 + 1);
            SB_ = (1.2*s112 - 1) / (s112 - 1);
            if (SB_ < SA_) SA_ = SB_;
            SB_ = S12_ / pow(SA_ - 1.2, 2);
        }
        Xd1_ = Xd_ - Xdp_;
        Xd2_ = Xdp_ - Xl_;
        Xd3_ = (Xdp_ - Xdpp_) / (Xd2_ * Xd2_);
        Xd4_ = (Xdp_ - Xdpp_) / Xd2_;
        Xd5_ = (Xdpp_ - Xl_) / Xd2_;
        Xq1_ = Xq_ - Xqp_;
        Xq2_ = Xqp_ - Xl_;
        Xq3_ = (Xqp_ - Xqpp_) / (Xq2_ * Xq2_);
        Xq4_ = (Xqp_ - Xqpp_) / Xq2_;
        Xq5_ = (Xqpp_ - Xl_) / Xq2_;
        Xqd_ = (Xq_ - Xl_) / (Xd_ - Xl_);
        gg_  = Ra_ / (Ra_*Ra_ + Xqpp_*Xqpp_);
        bb_  = -Xqpp_ / (Ra_*Ra_ + Xqpp_*Xqpp_);  
    }

    // Available template instantiations
    template class SynchronousMachine<double, long int>;
    template class SynchronousMachine<double, size_t>;

} //namespace PhasorDynamics
} //namespace GridKit
