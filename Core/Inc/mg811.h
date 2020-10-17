//
// Created by juan_ on 6/9/2020.
//

#ifndef TESTLCDI2C_MG811_H
#define TESTLCDI2C_MG811_H
#include "stm32f4xx_hal.h"

typedef struct
{
    double co2_a;
    double co2ppm;
    double inertia;
    int tries;
    double co2_v;
}MG811_DataTypeDef;

uint32_t MG811_Read(MG811_DataTypeDef *MG811_Data, uint32_t adc_value);

void MG811_Calibrate(MG811_DataTypeDef *MG811_Data, uint32_t adc_value);

void MG811_Init_Default(MG811_DataTypeDef *MG811_Data);

void MG811_Init(MG811_DataTypeDef *MG811_Data, int tries, double inertia);

#endif //TESTLCDI2C_MG811_H
