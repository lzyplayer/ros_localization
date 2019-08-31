#ifndef MODELS_BICYCLELR_HPP
#define MODELS_BICYCLELR_HPP

#include "ekf_core/extended_kalman_filter.hpp"
#include "ekf_core/kalman_filter_model_base.hpp"

struct BICYCLELRModelParameter : KalmanFilterModelParameterBase
{
    BICYCLELRModelParameter()
    {
        this->sigma2_beta = 0.01;
        this->sigma2_a = 0.1;
        this->sigma2_lr = 0;
        this->l = 2.85;
    }

    BICYCLELRModelParameter(double sigma2_beta, double sigma2_a, double sigma2_lr, double l)
    {
        this->sigma2_beta = sigma2_beta;
        this->sigma2_a = sigma2_a;
        this->sigma2_lr = sigma2_lr;
        this->l = 2.85;
    }

    double sigma2_beta;
    double sigma2_a;
    double sigma2_lr;

    double l;
};

class BICYCLELRModel : public KalmanFilterModelBase<7, BICYCLELRModelParameter>
{
public:
    static void initState(
        const ModelParameter& parameter,
        FilterVector& state,
        FilterMatrix& state_covariance
    )
    {
        state << 0, 0, 1.25, 0, 0, 0, 0;

        for(int i=0; i<7; i++)
        {
            state_covariance(i,i) = 99999;
        }

        state_covariance(2,2) = 0.01;
    }

    static void updatePrediction(
        const ModelParameter& parameter,
        const FilterVector& state,
        const double delta,
        FilterVector& state_prediction,
        FilterMatrix& jacobian,
        FilterMatrix& process_noise
    )
    {
        //std::cout << "param:" << parameter.sigma2_a<<std::endl;
        const double& X_ = state(0);
        const double& Y_ = state(1);
        const double& LR_= state(2);
        const double& T_ = state(3);
        const double& B_ = state(4);
        const double& V_ = state(5);
        const double& A_ = state(6);

        double& PX_ = state_prediction(0);
        double& PY_ = state_prediction(1);
        double& PLR_= state_prediction(2);
        double& PT_ = state_prediction(3);
        double& PB_ = state_prediction(4);
        double& PV_ = state_prediction(5);
        double& PA_ = state_prediction(6);

        const double LIMITB = 0.01;
        // const double parameter.sigma2_beta = 100.0;
        // const double parameter.sigma2_a = 100;

        double delta_1 = delta;
        double delta_2 = delta * delta;
        double delta_3 = delta * delta * delta; 
        double delta_4 = delta * delta * delta * delta; 
        
        double SB_ = sin(B_);
        double CB_ = cos(B_);
        double CSB_ = 1/sin(B_);
        double CTB_ = 1/tan(B_);
        double SBT_ = sin(B_ + T_);
        double CBT_ = cos(B_ + T_);
        double TAT2V_ = delta * (A_ * delta + 2 * V_);
        double TATS_2LR_ = (TAT2V_ * SB_)/(2.0 * LR_);
        double TATC_2LR_ = (TAT2V_ * CB_)/(2.0 * LR_);
        double SBTT_ = sin(B_ + T_ + TATS_2LR_);
        double CBTT_ = cos(B_ + T_ + TATS_2LR_);
        double L2RAT2_ = 2 * LR_ + A_ * delta_2 + 2 * delta * V_;
        double L2RAT2S_8LR2_ = (L2RAT2_ * L2RAT2_ * sin(T_))/(8.0 * LR_ * LR_);
        double L2RAT2C_8LR2_ = (L2RAT2_ * L2RAT2_ * cos(T_))/(8.0 * LR_ * LR_);
        double TV2_ = delta * V_ * V_;
        double S2BT_ = sin(2 * B_ + T_);
        double C2BT_ = cos(2 * B_ + T_);
        double AT3V_ = 2 * A_ * delta + 3 * V_;
        /*--------------------------------------------------------------------*\
        ** CALC State Predition by f(State)
        \*--------------------------------------------------------------------*/
        {
            if(abs(B_) < LIMITB)
            {
                PX_ = X_ + (1/2.0) * TAT2V_ * cos(T_) + LR_ * B_ * ( sin(T_)/2.0 - L2RAT2S_8LR2_);
                PY_ = Y_ + (1/2.0) * TAT2V_ * sin(T_) + LR_ * B_ * (-cos(T_)/2.0 + L2RAT2C_8LR2_);
            }else{
                PX_ = X_ - ((LR_ * CSB_) * (SBT_ - SBTT_));
                PY_ = Y_ + LR_ * (CBT_ - CBTT_) * CSB_;
            }
            
            PT_ = T_ + TATS_2LR_;
            PV_ = V_ + A_ * delta;
            PB_ = B_;
            PA_ = A_; 
            PLR_ = LR_;
            PT_ = NormalizeAngle(PT_);
            PB_ = NormalizeAngle(PB_);
        }
        /*--------------------------------------------------------------------*\
        ** CALC F Jacobian Matrix
        \*--------------------------------------------------------------------*/
        {
            double fxt = 0.0;
            double fyt = 0.0;
            double fxb = 0.0;
            double fyb = 0.0;
            double fxlr = 0.0;
            double fylr = 0.0;
            double ftlr = 0.0;
            if(abs(B_) < LIMITB)
            {
                fxlr = (B_ * (A_ * A_ * delta_4 * sin(T_) + 4 * A_ * delta_3 * V_ * sin(T_) + 4 * delta_2 * V_ * V_ * sin(T_)))/(8.0 * LR_ * LR_);
                fylr = (- B_ * (A_ * A_ * delta_4 * cos(T_) + 4 * A_ * delta_3 * V_ * cos(T_) + 4 * delta_2 * V_ * V_ * cos(T_)))/(8.0 * LR_ * LR_);
                fxt = (-1/2.0) * TAT2V_ * sin(T_) - (B_ * (cos(T_) * TAT2V_ * (TAT2V_ + 4 * LR_)))/(8.0 * LR_);
                fyt = ( 1/2.0) * TAT2V_ * cos(T_) - (B_ * (sin(T_) * TAT2V_ * (TAT2V_ + 4 * LR_)))/(8.0 * LR_) ;
                fxb = (-(TAT2V_ * (4 * LR_ + TAT2V_) * sin(T_))/(8.0 * LR_)) - ((TAT2V_ * (12 * LR_ * LR_ + 6 * LR_ * TAT2V_ + delta_2 * (A_ * delta + 2 * V_) * (A_ * delta + 2 * V_))* cos(T_)) * B_)/(24.0 * LR_ * LR_);
                fyb = ( (TAT2V_ * (4 * LR_ + TAT2V_) * cos(T_))/(8.0 * LR_)) - ((TAT2V_ * (12 * LR_ * LR_ + 6 * LR_ * TAT2V_ + delta_2 * (A_ * delta + 2 * V_) * (A_ * delta + 2 * V_))* sin(T_)) * B_)/(24.0 * LR_ * LR_);
                ftlr = (- B_ * TAT2V_)/(2.0 * LR_ * LR_);
            }else{
                fxlr = CSB_ * (SBTT_ - SBT_) - (TAT2V_ * CBTT_)/(2.0 * LR_);
                fylr = CSB_ * (CBT_ - CBTT_) - (TAT2V_ * SBTT_)/(2.0 * LR_);
                fxt = LR_ * CSB_ * (- CBT_ + CBTT_);
                fyt = LR_ * CSB_ * (- SBT_ + SBTT_);
                fxb = LR_ * CSB_ * (- CBT_ + (1 + TATC_2LR_) * CBTT_ + CTB_ * (SBT_ - SBTT_));
                fyb = (1/2.0) * CSB_ * (-2 * LR_ * ((CBT_ - CBTT_) * CTB_ + SBT_) + (2 * LR_ + TAT2V_ * CB_) * SBTT_);
                ftlr = -(TAT2V_ * SB_)/(2.0 * LR_ * LR_);
            }
            
            double fxv = delta * CBTT_;
            double fyv = delta * SBTT_;
            double ftv = delta * SB_/LR_;
            double ftb = (TAT2V_ * CB_)/(2.0 * LR_);
            double fxa = (1/2.0) * delta_2 * CBTT_;
            double fya = (1/2.0) * delta_2 * SBTT_;
            double fta = (delta_2 * SB_)/(2.0 * LR_);
            double fva = delta_1;
            
            jacobian <<    1,   0, fxlr ,fxt, fxb, fxv, fxa,
                           0,   1, fylr ,fyt, fyb, fyv, fya,
                           0,   0,    1,   0,   0,   0,   0,
                           0,   0, ftlr,   1,  ftb, ftv, fta,
                           0,   0,   0 ,   0,   1,   0,   0,
                           0,   0,   0 ,   0,   0,   1, fva,
                           0,   0,   0 ,   0,   0,   0,   1;

        }
        /*--------------------------------------------------------------------*\
        ** CALC Process Noice Q Matrix
        \*--------------------------------------------------------------------*/
        {
            // double XGB_ = -(delta_3 * V_ * V_ * CB_ * SBT_)/(6.0 * LR_);
            // double XGA_ = (1/6.0) * delta_3 * CBT_;
            // double YGB_ = (delta_3 * V_ * V_ * CB_ * CBT_)/(6.0 * LR_);
            // double YGA_ = (1/6.0) * delta_3 * SBT_;
            // double TGB_ = (V_ * CB_ * delta_2)/(2.0 * LR_);
            // double TGA_ = (SB_ * delta_3)/(6.0 * LR_);

            double XGB_ = (delta_2 * (-2 * LR_ * AT3V_ * SBT_ - 3 * TV2_ * S2BT_ + TV2_ * sin(T_)))/(12.0 * LR_);
            double XGA_ = (1/6.0) * delta_3 * CBT_;
            double YGB_ = (delta_2 * ( 2 * LR_ * AT3V_ * CBT_ + 3 * TV2_ * C2BT_ - TV2_ * cos(T_)))/(12.0 * LR_);
            double YGA_ = (1/6.0) * delta_3 * SBT_;
            double TGB_ = (2 * A_ * delta_1 * CB_)/(LR_ * parameter.sigma2_beta);
            double TGA_ = (SB_ * delta_1)/(LR_ * parameter.sigma2_beta); 
            
            // Eigen::Matrix<double, 7, 3> gamma_;
            // Eigen::Matrix<double, 3, 3> sigma_;
            // gamma_ <<
            //           0,      XGB_,                XGA_,
            //           0,      YGB_,                YGA_, 
            //           delta_1,   0,                   0,
            //           0,      TGB_,                TGA_,
            //           0,   delta_1,                   0,
            //           0,         0,  ( 1/2.0) * delta_2,
            //           0,         0,             delta_1;              
                 
            // sigma_ <<parameter.sigma2_lr,                     0,         0,
            //                0,  parameter.sigma2_beta ,         0, 
            //                0,              0, parameter.sigma2_a;

            Eigen::Matrix<double, 7, 2> gamma_;
            Eigen::Matrix<double, 2, 2> sigma_;
            gamma_ <<
                          XGB_,                XGA_,
                          YGB_,                YGA_, 
                             0,                   0,
                          TGB_,                TGA_,
                       delta_1,                   0,
                             0,  ( 1/2.0) * delta_2,
                             0,             delta_1;              
                 
            sigma_ <<     parameter.sigma2_beta ,                  0, 
                                               0, parameter.sigma2_a;

            process_noise = gamma_ * sigma_ * gamma_.transpose();
        }

        //std::cout<<"beta:"<<parameter.sigma2_beta<<"      a:"<<parameter.sigma2_a<<std::endl;
    }
};

#endif