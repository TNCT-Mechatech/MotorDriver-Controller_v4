/*
 * TwoWireMD.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Taiyou Komazawa
 */

#ifndef SRC_TWOWIREMD_HPP_
#define SRC_TWOWIREMD_HPP_

#include <math.h>

#include <main.h>
#include <MotorDriver.hpp>

/**
 * @brief 2線式(DIR,PWM)制御のMotorDriver派生クラス
 */
class TwoWireMD final : public MotorDriver
{
public:
    TwoWireMD(TIM_HandleTypeDef *htim_pwm, uint16_t tim_pwm_ch,
              GPIO_TypeDef *dir_port, uint16_t dir_pin,
              bool inverse_dir=false,
              float max_power = 1.0);
    ~TwoWireMD();

    virtual void set(float power);
    void set(unsigned int pwm, bool dir);
private:

    TIM_HandleTypeDef *htim_pwm_;
    uint16_t tim_pwm_ch_;
    GPIO_TypeDef *dir_port_;
    uint16_t dir_pin_;

    bool inverse_dir_;

    unsigned int limit_;
};

#endif /* SRC_TWOWIREMD_HPP_ */
