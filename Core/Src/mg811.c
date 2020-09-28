//
// Created by juan_ on 6/9/2020.
//

#include "mg811.h"
#include <math.h>
#include "stm32f4xx_hal.h"

const int co2_b = 600;
const int co2_d = 400;
uint16_t adc_value; // variable for reading the ADC value

#define MG811_PORT GPIOA
#define MG811_PIN GPIO_PIN_1

#define CO2_LOW 600
#define CO2_HIGHT 1000

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    adc_value = HAL_ADC_GetValue(hadc); // read ADC value
    HAL_ADC_Start_IT(hadc);
}

uint16_t readPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, ADC_HandleTypeDef* hadc){
    HAL_ADC_Start_IT(hadc);
    return adc_value;
}


uint32_t MG811_Read(MG811_DataTypeDef *MG811_Data, ADC_HandleTypeDef* hadc){
    int v = 0;

    readPin(MG811_PORT, MG811_PIN, hadc);
    for (int i = 0; i < MG811_Data->tries; i++) {
        v += readPin(MG811_PORT, MG811_PIN, hadc);
        HAL_Delay(20);
    }
    MG811_Data->co2_v = (1 - MG811_Data->inertia) * (v * 5000.0) / (1024.0 * MG811_Data->tries) + MG811_Data->co2_v * MG811_Data->inertia;

    double co2_exp = (MG811_Data->co2_a - MG811_Data->co2_v) / co2_b;
    MG811_Data->co2ppm = pow(co2_d, co2_exp);

//    return MG811_Data->co2ppm;
    return adc_value;
}

void MG811_Calibrate(MG811_DataTypeDef *MG811_Data, ADC_HandleTypeDef* hadc) {
    MG811_Read(MG811_Data, hadc);
    MG811_Data->co2_a = MG811_Data->co2_v + co2_b;
    MG811_Data->co2ppm = co2_d;
}

void MG811_Init_Default(MG811_DataTypeDef *MG811_Data) {
    MG811_Data->co2_a = 1500;
    MG811_Data->co2ppm = co2_d;
}

void MG811_Init(MG811_DataTypeDef *MG811_Data, int d_tries, double d_inertia) {
    MG811_Data->tries = d_tries;
    MG811_Data->inertia = d_inertia;
    MG811_Data->co2_a = 1500;
    MG811_Data->co2ppm = co2_d;
}
