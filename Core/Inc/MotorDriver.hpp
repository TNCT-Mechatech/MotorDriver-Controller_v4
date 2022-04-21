/*
 * MotorDriver.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Taiyou Komazawa
 */

#ifndef INC_MOTORDRIVER_HPP_
#define INC_MOTORDRIVER_HPP_

/**
 * @brief モータドライバの仮想クラス
 */
class MotorDriver
{
public:
    MotorDriver();
    virtual ~MotorDriver();
    /**
     * @brief 出力値を指定する関数(1.0~-1.0)
     * @retval None
     */
    virtual void set(float) = 0;
};

#endif /* INC_MOTORDRIVER_HPP_ */
