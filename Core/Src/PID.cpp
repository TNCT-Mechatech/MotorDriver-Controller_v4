/*
 * PID.cpp
 *
 *  Created on: Nov 8, 2021
 *      Author: Taiyou Komazawa
 */

#include "PID.hpp"
/**
 * @brief Construct a new PID::PID object
 *
 * @param[in out] cv PID::ctrl_variable_tのポインタ変数
 * @param[in] 	  cp PID::ctrl_param_tのポインタ変数
 */
PID::PID(PID::ctrl_variable_t *cv, PID::ctrl_param_t *cp)
  : cv_(cv), cp_(cp)
{
  cv_->target = 0;
  cv_->feedback = 0;
  PID::reset();
}

/**
 * @brief Destroy the PID::PID object
 *
 */
PID::~PID()
{
  cv_->target = 0;
  cv_->feedback = 0;
  PID::reset();
}

/**
 * @brief PID制御器のリセットを行う
 * @retval None
 */
void PID::reset()
{
  last_diff_ = 0;
  last_feedback_ = 0;
  integral_ = 0;
  cv_->output = 0;
}

/**
 * @brief PID制御器を1ステップ更新する
 *
 * @param[in] dt 制御間隔[sec]
 * @retval None
 */
void PID::step(double dt)
{
  double diff = cv_->target - cv_->feedback;
  cv_->output = cp_->forward_gain * cv_->target + cp_->kp * diff;
  if(cp_->ki){
    integral_ += (diff + last_diff_) / 2 * dt;
    cv_->output += cp_->ki * integral_;
  }
  if(cp_->kd && dt){
    if(cp_->dpi_mode){
      cv_->output -= cp_->kd * (cv_->feedback - last_feedback_) / dt;
      last_feedback_ = cv_->feedback;
    }else
      cv_->output += cp_->kd * (diff - last_diff_) / dt;
  }

  last_diff_ = diff;
}
