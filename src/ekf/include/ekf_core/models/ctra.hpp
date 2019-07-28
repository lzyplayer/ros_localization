#ifndef MODELS_CTRA_HPP
#define MODELS_CTRA_HPP

#include "ekf_core/kalman_filter_model_base.hpp"

struct CTRAModelParameter : KalmanFilterModelParameterBase//结构体的继承，默认公有继承
{
    CTRAModelParameter()
    {
        this->sigma2_omega = 100;
        this->sigma2_a = 100;
        this->l=2.85;
        this->W=1.5;
    }

    CTRAModelParameter(double sigma2_omega, double sigma2_a, double l, double W)
    {
        this->sigma2_omega = sigma2_omega;
        this->sigma2_a = sigma2_a;
        this->l=l;
        this->W=1.5;
    }

    double sigma2_omega;
    double sigma2_a;
    double l;
    double W;
};

class CTRAModel : public KalmanFilterModelBase<6, CTRAModelParameter>//公有继承，父类是生成的模板类
{
public:
    static void initState(
        const ModelParameter& parameter,
        FilterVector& state,
        FilterMatrix& state_covariance
    )
    {
        // state << 0, 0, 3.10424958891418, 3.0, 0.00493377493694, 0.0389796892898;
        state << 0, 0, 0, 0, 0, 0;
        for(int i=0; i<6; i++)
        {
            state_covariance(i,i) = 99999;//P
        }
    }

    static void updatePrediction(//计算出一步预测，包括F矩阵，，，，，，，，，，，
        const ModelParameter& parameter,
        const FilterVector& state,
        const double delta,
        FilterVector& state_prediction,
        FilterMatrix& jacobian,
        FilterMatrix& process_noise
    )
    {
        const double& X_ = state(0);
        const double& Y_ = state(1);
        const double& T_ = state(2);
        const double& V_ = state(3);
        const double& W_ = state(4);
        const double& A_ = state(5);

        double& PX_ = state_prediction(0);//引用，相当于同一个东西两个名字而已
        double& PY_ = state_prediction(1);
        double& PT_ = state_prediction(2);
        double& PV_ = state_prediction(3);
        double& PW_ = state_prediction(4);
        double& PA_ = state_prediction(5);

        const double LIMITW = 0.01;
        // const double parameter.sigma2_omega = 1;
        // const double parameter.sigma2_a = 500;

        double txv, tyv, t2xa, t2ya, Ttxv_w, Ttyv_w;
        double delta_1 = delta;
        double delta_2 = delta * delta;
        double delta_3 = delta * delta * delta; 
        if(abs(W_) > LIMITW)
        {
            double ypwT = T_ + W_*delta;
            double w_1  = 1.0/ W_;
            double w_2  = 1.0/(W_ * W_);

            txv  =   w_1 * (sin(ypwT) - sin(T_));
            tyv  = - w_1 * (cos(ypwT) - cos(T_));
            t2xa = 2 * (  delta * w_1 * sin(ypwT) + w_2 * (cos(ypwT)-cos(T_)));
            t2ya = 2 * (- delta * w_1 * cos(ypwT) + w_2 * (sin(ypwT)-sin(T_)));
            Ttxv_w = (delta * txv - t2xa)/W_;
            Ttyv_w = (delta * tyv - t2ya)/W_;
        }
        else
        {
            txv =               delta_1 * cos(T_);
            tyv =               delta_1 * sin(T_);
            t2xa =              delta_2 * cos(T_);
            t2ya =              delta_2 * sin(T_);
            Ttxv_w = ( 1/6.0) * delta_3 * sin(T_);
            Ttyv_w = (-1/6.0) * delta_3 * cos(T_);
        }
        /*--------------------------------------------------------------------*\
        ** CALC State Predition by f(State)
        \*--------------------------------------------------------------------*/
        {
            PX_ = X_ + V_ * txv + A_ * t2xa / 2.0;
            PY_ = Y_ + V_ * tyv + A_ * t2ya / 2.0;
            PT_ = T_ + W_ * delta;
            PV_ = V_ + A_ * delta;
            PW_ = W_;
            PA_ = A_;
        }
        /*--------------------------------------------------------------------*\
        ** CALC F Jacobian Matrix
        \*--------------------------------------------------------------------*/
        {
            double fxt = -(V_ * tyv + A_ * t2ya / 2.0);
            double fyt = +(V_ * txv + A_ * t2xa / 2.0);
            double fxv = txv;
            double fyv = tyv;
            double fxw = A_ * Ttxv_w - (V_ + A_ * delta) * t2ya / 2.0;
            double fyw = A_ * Ttyv_w + (V_ + A_ * delta) * t2xa / 2.0;
            double fxa = t2xa / 2.0;
            double fya = t2ya / 2.0;

            double ftw = delta;
            double fva = delta;
            
            jacobian <<    1,   0, fxt, fxv, fxw, fxa,
                        0,   1, fyt, fyv, fyw, fya,
                        0,   0,   1,   0, ftw,   0,
                        0,   0,   0,   1,   0, fva,
                        0,   0,   0,   0,   1,   0,
                        0,   0,   0,   0,   0,   1;
        }
        /*--------------------------------------------------------------------*\
        ** CALC Process Noice Q Matrix
        \*--------------------------------------------------------------------*/
        {
            double CT_    = cos(T_);
            double ST_    = sin(T_);
            double VCT_   = V_*CT_;
            double VST_   = V_*ST_;
            Eigen::Matrix<double, 6, 2> gamma_;
            Eigen::Matrix<double, 2, 2> sigma_;
            gamma_ <<
                (-1/6.0) * delta_3 * VST_,  ( 1/6.0) * delta_3 * CT_,
                ( 1/6.0) * delta_3 * VCT_,  ( 1/6.0) * delta_3 * ST_, 
                ( 1/2.0) * delta_2       ,                         0,
                                        0,  ( 1/2.0) * delta_2      ,
                        delta_1       ,                         0,
                                        0,             delta_1      ;                    
            sigma_ <<
                parameter.sigma2_omega ,        0, 
                            0, parameter.sigma2_a;
            process_noise = gamma_ * sigma_ * gamma_.transpose();
        }
    }
};

#endif