//
// Created by testusuke on 2023/07/20.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_THERMALSENSOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_THERMALSENSOR_HPP

#include "main.h"
#include "math.h"
#include "SimpleMovingAverage.hpp"

#define SMA_K 1

#define RESOLUTION 4096

#define B_CONSTANT 4101             //  [K]
#define BASE_TEMP 298.15            //  [K]
#define BASE_RESISTANCE 47000       //  [Ω]
#define CIRCUIT_RESISTANCE  10000   //  [Ω]
#define VOLTAGE_IN  3.3             //  [V]

class ThermalSensor
{
public:
    ThermalSensor(ADC_HandleTypeDef *hadcx);
    ~ThermalSensor();

    double get_temperature();

private:
    ADC_HandleTypeDef *hadcx_;
    uint32_t _raw;

    const double B_INVERSE = 1.0 / (double)B_CONSTANT;
    const double BASE_TEMP_INVERSE = 1.0 / (double)BASE_TEMP;

    SimpleMovingAverage<double> _sma;
};
#endif //MOTORDRIVER_CONTROLLER_V4_THERMALSENSOR_HPP
