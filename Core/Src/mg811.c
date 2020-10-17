//
// Created by juan_ on 6/9/2020.
//

#include "mg811.h"
#include <math.h>
#include "stm32f4xx_hal.h"

const int co2_b = 600;
const int co2_d = 400;
//uint16_t adc_value; // variable for reading the ADC value

//#define MG811_ADC_CHANNEL ADC_CHANNEL_1

#define CO2_LOW 600
#define CO2_HIGHT 1000

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//    adc_value = HAL_ADC_GetValue(hadc); // read ADC value
//    HAL_ADC_Start_IT(hadc);
//}

//uint16_t readPin(uint32_t ADC_CHANNEL, ADC_HandleTypeDef* hadc){
////    ADC_ChannelConfTypeDef channelConfig = {0};
////    channelConfig.Channel = ADC_CHANNEL;
////    channelConfig.Rank = 1;
////    channelConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
////    HAL_ADC_ConfigChannel(hadc, &channelConfig);
//    adc_value = HAL_ADC_GetValue(hadc);
//    HAL_ADC_Start_IT(hadc);
//    return adc_value;
//}


uint32_t MG811_Read(MG811_DataTypeDef *MG811_Data, uint32_t adc_value){
    float v = adc_value;
//    for (int i = 0; i < MG811_Data->tries; i++) {
//        v += readPin(MG811_ADC_CHANNEL, hadc);
//        HAL_Delay(20);
//    }
    MG811_Data->co2_v = (1 - MG811_Data->inertia) * v * 2000 / 4096.0 + MG811_Data->co2_v * MG811_Data->inertia;

    double co2_exp = (MG811_Data->co2_a - MG811_Data->co2_v) / co2_b;
    MG811_Data->co2ppm = pow(co2_d, co2_exp);

    return MG811_Data->co2ppm; //v;
//    return adc_value;
}

void MG811_Calibrate(MG811_DataTypeDef *MG811_Data, uint32_t adc_value) {
    MG811_Read(MG811_Data, adc_value);
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
