//
// Created by owner on 2023/07/22.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_CURRENTSENSOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_CURRENTSENSOR_HPP

#include "main.h"
#include "SimpleMovingAverage.hpp"
#include "array"

#define SMA_K 3
#define RESOLUTION 4096

#define VOLTAGE_IN  3.3
#define ZERO_CURRENT_VOLTAGE 1.65   //  [V]
#define CURRENT_SENSE 48            //  [mV/A]

class CurrentSensor
{
public:
    CurrentSensor(ADC_HandleTypeDef *hadcx, int unit_num);
    ~CurrentSensor();

    double get_current(int unit);

private:
    ADC_HandleTypeDef *hadcx_;
    int _raw_num;
    uint16_t *_raw;

    std::array<SimpleMovingAverage<double>*, 8> sma_array;
};

#endif //MOTORDRIVER_CONTROLLER_V4_CURRENTSENSOR_HPP
