#ifndef PID_H
#define PID_H

#include "Arduino.h"

// PID 类头文件
class PID
{
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double compute(float setpoint, float measured_value); //计算输出
        void updateConstants(float kp, float ki, float kd); //更新pid参数

    private:
        float min_val_; //最小输出，根据微控制器的位宽设置
        float max_val_; //最大输出
        float kp_;
        float ki_;
        float kd_;
        double integral_; //积分
        double derivative_; //微分
        double prev_error_; //比例
};

#endif
