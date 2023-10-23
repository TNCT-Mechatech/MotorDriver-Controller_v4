/*
 * QEI.hpp
 *
 *  Created on: Jan 2, 2022
 *      Author: Taiyou Komazawa
 */

#ifndef INC_QEI_HPP_
#define INC_QEI_HPP_

#include "main.h"

/**
 * @brief 直交エンコーダインターフェイスクラス.
 * STM32タイマーのEncoderModeを用いて2相式エンコーダから角度及び速度を取得する
 */
class QEI
{
public:
    QEI(TIM_HandleTypeDef *htim_enc, double scale=1.0);

    ~QEI();

    void reset_count();
    void set_scale(double scale);

    double get_angle();

    double get_velocity(double dt);
    double get_velocity();

private:
    TIM_HandleTypeDef *htim_enc_;
    volatile long count_;
    double scale_;
    double last_angle_;
    double vel_;
};

#endif /* INC_QEI_HPP_ */
