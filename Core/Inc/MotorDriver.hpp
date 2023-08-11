#ifndef _MDC_MOTOR_DRIVER_HPP_
#define _MDC_MOTOR_DRIVER_HPP_

#include <math.h>
#include <main.h>

class MotorDriver
{
public:
    MotorDriver(TIM_HandleTypeDef *htim_pwm, uint16_t tim_pwm_ch,
              GPIO_TypeDef *dir_port, uint16_t dir_pin,
              bool inverse_dir=false,
              float max_power = 1.0,
              float min_power = 0.0);
    ~MotorDriver();

    void set(float power);
    void set(unsigned int pwm, bool dir);
    void set_inverse_dir(bool inverse_dir);
private:

    TIM_HandleTypeDef *htim_pwm_;
    uint16_t tim_pwm_ch_;
    GPIO_TypeDef *dir_port_;
    uint16_t dir_pin_;
    bool inverse_dir_;
    unsigned int limit_;
    unsigned int min_power_;
};

#endif /* _MDC_MOTOR_DRIVER_HPP_ */
