/*
 * QEI.cpp
 *
 *  Created on: Jan 2, 2022
 *      Author: Taiyou Komazawa
 */

#include <QEI.hpp>

/**
 * @brief Construct a new QEI::QEI object
 *
 * @param[in] htim_enc エンコーダを使用するTIM_HandleTypeDef構造体ポインタ
 * @param[in] scale    エンコーダの角度スケール(ギア比とかCPR)
 */
QEI::QEI(TIM_HandleTypeDef *htim_enc, double scale)
  :	htim_enc_(htim_enc),
     scale_(scale),
     vel_(0)
{
  if(HAL_TIM_Encoder_Start(htim_enc_, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();
  reset_count();
  last_angle_ = get_angle();
}

/**
 * @brief Destroy the QEI::QEI object
 */
QEI::~QEI()
{
  if(HAL_TIM_Encoder_Stop(htim_enc_, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();
  reset_count();
}

/**
 * @brief カウンタをリセットする関数
 */
void QEI::reset_count()
{
  count_ = 0;
  __HAL_TIM_GET_COUNTER(htim_enc_) = 0;
}

/**
 * @brief set scale
 */
 void QEI::set_scale(double scale) {
     scale_ = scale;
 }

/**
 * @brief 現在の角度を出力する関数
 * @return double 現在の角度(カウント値 * scale) [Rotation]
 */
double QEI::get_angle()
{
  int16_t enc_buff = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_);
  __HAL_TIM_GET_COUNTER(htim_enc_) = 0;
  count_ += enc_buff;
  return count_ * scale_;
}

/**
 * @brief 現在の速度を出力する関数(get_angle()の微分)
 *
 * @param[in] dt 測定間隔[sec]
 * @return double 現在の速度(角度の微分値) [RPS]
 */
double QEI::get_velocity(double dt)
{
  if(dt){
    vel_ = (get_angle() - last_angle_) / dt;
    last_angle_ = get_angle();
    return vel_;
  }
  return 0;
}

/**
 * @brief 速度を出力する関数(データを更新しない)
 *
 * @return double 最新のQEI::get_velocity(double dt)の値
 */
double QEI::get_velocity()
{
  return vel_;
}
