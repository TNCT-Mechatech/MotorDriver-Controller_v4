//
// Created by testusuke on 2023/07/20.
//

#include "ThermalSensor.hpp"

ThermalSensor::ThermalSensor(ADC_HandleTypeDef *hadcx)
    : hadcx_(hadcx), _sma(SimpleMovingAverage<double>(window_number(SMA_K), 0))
{
    if (HAL_ADC_Start_DMA(hadcx_, (uint32_t *) _raw, 1) != HAL_OK) {
        Error_Handler();
    }
}

ThermalSensor::~ThermalSensor()
{
    if(HAL_ADC_Stop_DMA(hadcx_) != HAL_OK) {
        Error_Handler();
    }
}

double ThermalSensor::get_temperature() {
    //  calculate
    double measured_voltage = (float)_raw / RESOLUTION * VOLTAGE_IN;
    double resistance = (double) BASE_RESISTANCE / (VOLTAGE_IN / measured_voltage - 1.0);
    double inverse_temp = B_INVERSE * log(resistance / BASE_RESISTANCE) + BASE_TEMP_INVERSE;
    double temp = 1.0 / inverse_temp;

    //  filter
    _sma.push(temp);

    return _sma.calculate();
}
