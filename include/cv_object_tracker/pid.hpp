#ifndef CV_OBJECT_TRACKER__PID_HPP_
#define CV_OBJECT_TRACKER__PID_HPP_

#include <algorithm>

namespace cv_tracker {

    class  PID {
        public:
            PID(double kp, double ki, double kd, double min_out, double max_out)
            :kp_(kp), ki_(ki), kd_(kd), min_(min_out), max_(max_out), prev_error_(0), integral_(0), first_run_(true){}
            
            void reset(){
                prev_error_= 0;
                integral_= 0; 
                first_run_ = true;
            }
            double calculate(double setpoint, double measurement, double dt){
                if (dt <= 0) {
                    return 0;
                }

                double error = setpoint - measurement;
                double P = kp_ * error;

                integral_ += error * dt;
                double I = ki_ * integral_;
                
                double D = 0;
                if (!first_run_){
                    D = kd_ * (error - prev_error_) / dt;
                }

                prev_error_ = error ;
                first_run_ = false ;

                return std::clamp( P + I + D , min_, max_);
            }
        private:
            double kp_,ki_,kd_;
            double min_,max_;
            double prev_error_, integral_;
            bool first_run_;
    };
}//namespace

#endif