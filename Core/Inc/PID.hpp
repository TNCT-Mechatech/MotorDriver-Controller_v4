/*
 * PID.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: Taiyou Komazawa
 */

#ifndef INC_PID_HPP_
#define INC_PID_HPP_

#include <stdint.h>

/**
 * @brief 汎用的なPID制御を実行するクラス
 */
class PID
{
public:
    /**
     * @brief 制御で扱う変数構造型
     */
    typedef struct ControllerVariableType
    {
        /* 制御指示値 */
        double target;
        /* フィードバック値 */
        double feedback;
        /* 制御出力 */
        double output;
    }ctrl_variable_t;
    /**
     * @brief 制御で扱うパラメータ構造体型
     */
    typedef struct ControllerParameterType
    {
        /* 比例制御ゲイン */
        float 	kp = 0.1;
        /* 積分制御ゲイン */
        float 	ki = 0;
        /* 微分制御ゲイン */
        float 	kd = 0;
        /* フィードフォワードゲイン */
        float 	forward_gain = 0;
        /* 微分先行型(D-PI)制御モードを有効にする */
        bool 	dpi_mode = true;
    }ctrl_param_t;

    PID(ctrl_variable_t *cv, ctrl_param_t *cp);
    ~PID();

    void reset();

    void step(double dt);

private:
    ctrl_variable_t *cv_;
    ctrl_param_t *cp_;

    double last_diff_, integral_;
    double last_feedback_;
};


#endif /* INC_PID_H_ */
