//
// Created by juan_ on 6/9/2020.
//

#include "mg811.h"
#include <math.h>
#include "stm32f4xx_hal.h"
//
//const int co2_b = 600;
//const int co2_d = 400;
////uint16_t adc_value; // variable for reading the ADC value
//
////#define MG811_ADC_CHANNEL ADC_CHANNEL_1
//
//#define CO2_LOW 600
//#define CO2_HIGHT 1000
//
////void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
////    adc_value = HAL_ADC_GetValue(hadc); // read ADC value
////    HAL_ADC_Start_IT(hadc);
////}
//
////uint16_t readPin(uint32_t ADC_CHANNEL, ADC_HandleTypeDef* hadc){
//////    ADC_ChannelConfTypeDef channelConfig = {0};
//////    channelConfig.Channel = ADC_CHANNEL;
//////    channelConfig.Rank = 1;
//////    channelConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
//////    HAL_ADC_ConfigChannel(hadc, &channelConfig);
////    adc_value = HAL_ADC_GetValue(hadc);
////    HAL_ADC_Start_IT(hadc);
////    return adc_value;
////}
//
//
//uint32_t MG811_Read(MG811_DataTypeDef *MG811_Data, uint32_t adc_value){
//    float v = adc_value;
////    for (int i = 0; i < MG811_Data->tries; i++) {
////        v += readPin(MG811_ADC_CHANNEL, hadc);
////        HAL_Delay(20);
////    }
//    MG811_Data->co2_v = (1 - MG811_Data->inertia) * v * 2000 / 4095.0 + MG811_Data->co2_v * MG811_Data->inertia;
//
//    double co2_exp = (MG811_Data->co2_a - MG811_Data->co2_v) / co2_b;
//    MG811_Data->co2ppm = pow(co2_d, co2_exp);
//
//    return MG811_Data->co2ppm; //v;
////    return adc_value;
//}
//
//void MG811_Calibrate(MG811_DataTypeDef *MG811_Data, uint32_t adc_value) {
//    MG811_Read(MG811_Data, adc_value);
//    MG811_Data->co2_a = MG811_Data->co2_v + co2_b;
//    MG811_Data->co2ppm = co2_d;
//}
//
//void MG811_Init_Default(MG811_DataTypeDef *MG811_Data) {
//    MG811_Data->co2_a = 1500;
//    MG811_Data->co2ppm = co2_d;
//}
//
//void MG811_Init(MG811_DataTypeDef *MG811_Data, int d_tries, double d_inertia) {
//    MG811_Data->tries = d_tries;
//    MG811_Data->inertia = d_inertia;
//    MG811_Data->co2_a = 1500;
//    MG811_Data->co2ppm = co2_d;
//}



/************************Hardware Related Macros************************************/
#define         MG_PIN                       0     //define which analog input channel you are going to use
#define         BOOL_PIN                     2
#define         DC_GAIN                      8.5   //define the DC gain of amplifier

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         50    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            5     //define the time interval(in milisecond) between each samples in
//normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           0.250f //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             0.160f //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602f,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602f-3.0f))};
                                                                        ///two points are taken from the curve.
                                                                        ///with these two points, a line is formed which is
                                                                        ///"approximately equivalent" to the original curve.
                                                                        ///data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280)
                                                                        ///slope = ( reaction voltage ) / (log400 –log1000)







/*****************************  MGRead *********************************************
Input:   mg_pin - analog channel
Output:  output of SEN-000007
Remarks: This function reads the output of SEN-000007
************************************************************************************/
float MGRead(float adc_value)
{

#define INPUT_START 0.0f
#define INPUT_END 4095.0f
#define OUTPUT_START 0.0f
#define OUTPUT_END 3.0f

//    if (adc_value > INPUT_START) adc_value = INPUT_START;
//    if (adc_value <= INPUT_END) adc_value = INPUT_END + 1;
    return OUTPUT_START + ((OUTPUT_END - OUTPUT_START) / (INPUT_END - INPUT_START)) * (adc_value - INPUT_START);

//    for (i=0;i<READ_SAMPLE_TIMES;i++) {
//        v += analogRead(mg_pin);
//        delay(READ_SAMPLE_INTERVAL);
//    }
    adc_value = (adc_value) * 3.3f/4095.0f ;
    return adc_value;
}

/*****************************  MQGetPercentage **********************************
Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(MG-811 output) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MGGetPercentage(float volts)
{
    if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
        return -1;
    } else {
        return pow(10, ((volts/DC_GAIN)-CO2Curve[1])/CO2Curve[2]+CO2Curve[0]);
    }
}