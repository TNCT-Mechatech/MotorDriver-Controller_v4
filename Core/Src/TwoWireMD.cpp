/*
 * TwoWireMD.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Taiyou Komazawa
 */

#include "TwoWireMD.hpp"

/**
 * @brief Construct a new Two Wire MD:: Two Wire MD object
 *
 * @param[in] htim_pwm 		PWM出力用TIM_HandleTypeDef構造体ポインタ
 * @param[in] tim_pwm_ch	PWM出力先チャンネル
 * @param[in] dir_port		DIR(方向)出力用GPIO_TypeDef構造体ポインタ
 * @param[in] dir_pin 		DIR(方向)出力先ピン
 * @param[in] inverse_dir 	DIR(方向)出力を反転する
 */
TwoWireMD::TwoWireMD(TIM_HandleTypeDef *htim_pwm, uint16_t tim_pwm_ch,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     bool inverse_dir)
  : MotorDriver(),
    htim_pwm_(htim_pwm),
    tim_pwm_ch_(tim_pwm_ch),
    dir_port_(dir_port),
    dir_pin_(dir_pin),
    inverse_dir_(inverse_dir),
    limit_(htim_pwm->Init.Period)
{
  HAL_TIM_PWM_Start(htim_pwm_, tim_pwm_ch_);
  __HAL_TIM_SET_COMPARE(htim_pwm_, tim_pwm_ch_, 0);
  TwoWireMD::set(0);
}

/**
 * @brief Destroy the Two Wire MD:: Two Wire MD object
 */
TwoWireMD::~TwoWireMD()
{
  TwoWireMD::set(0);
}

/**
 * @brief MD出力を指定する関数
 *
 * @param[in] power モータ出力(-1.0~1.0)
 * @retval None
 */
void TwoWireMD::set(float power)
{
  /* powerが正のときDIRピンはTrue */
  bool dir = power > 0;
  TwoWireMD::set(abs((int)(power*limit_)), dir);
}

/**
 * @brief MD出力を指定する関数
 *
 * @param[in] pwm   モータへの信号のデューティ比(分解能はhtim_pwm->Init.Period依存)
 * @param[in] dir	DIRピンの状態(true:HIGH,false:LOW ただしinverse_dirにより反転)
 * @retval None
 */
void TwoWireMD::set(unsigned int pwm, bool dir)
{
  HAL_GPIO_WritePin(dir_port_, dir_pin_, (GPIO_PinState)(dir^inverse_dir_));
  if(limit_ < pwm)
    pwm = limit_;
  __HAL_TIM_SET_COMPARE(htim_pwm_, tim_pwm_ch_, pwm);
}
