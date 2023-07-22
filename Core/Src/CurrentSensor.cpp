//
// Created by testusuke on 2023/07/22.
//

#include "CurrentSensor.hpp"

CurrentSensor::CurrentSensor(ADC_HandleTypeDef *hadcx, int unit_num)
        : hadcx_(hadcx), _raw_num(unit_num), _raw(new uint16_t[unit_num])
{
    //  init sma
    for (int i = 0; i < _raw_num; ++i) {
        sma_array[i] = new SimpleMovingAverage<float>(window_number(SMA_K), 0.0);
    }

    if (HAL_ADC_Start_DMA(hadcx_, (uint32_t *) _raw, 1) != HAL_OK) {
        Error_Handler();
    }
}

CurrentSensor::~CurrentSensor()
{
    if(HAL_ADC_Stop_DMA(hadcx_) != HAL_OK) {
        Error_Handler();
    }
}

float CurrentSensor::get_current(int unit)
{
    float measured_voltage = (float)_raw[unit] / RESOLUTION * VOLTAGE_IN;
    float current = CURRENT_SENSE_INVERSE * (measured_voltage - ZERO_CURRENT_VOLTAGE) * 1000.0;

    //  sma
    sma_array[unit]->push(current);

    return sma_array[unit]->calculate();
}