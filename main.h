/*
 * main.h
 *
 *  Created on: Nov 6, 2016
 *      Author: Sayan
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_chip.h"
#include "em_acmp.h"
#include "dmactrl.h"
#include "em_adc.h"
#include "em_dma.h"
#include "em_int.h"
#include "em_i2c.h"
#include "em_leuart.h"


/***options***/
#define ENERGY_MODE EM2

#define CALIB_ON 1
#define I2C_0_ACMP_1 0
#define DMA_STATUS 1
/****** LETIMER constants*****/
#define LFXOFreq 32768
#define ULFRCOFreq 1000
#define LED_PORT    gpioPortE
#define LED_LIGHT_PIN     2
#define LED_TEMP_PIN     3
//#define Period     6.25
#define ON_DUTY_CYCLE     0.004
/*Pins*/
#define LIGHT_SENSE_PORT    gpioPortC
#define LIGHT_SENSE_PIN     6
#define LIGHT_EXCITE_PORT    gpioPortD
#define LIGHT_EXCITE_PIN     6
#define UF_Count 0x0000
#define Transmit_Port gpioPortD
#define Transmit_Pin 4
/*Temperature Limits*/
#define lowTempLimit 15
#define highTempLimit 35
/*ACMP inputs*/
#define ACMP_NEG_CHANNEL acmpChannelVDD
#define ACMP_POS_CHANNEL acmpChannel6

/*Slave register addresses*/
#define I2C_CTRL_ADD 0x00
#define I2C_TIMING_ADD 0x01
#define I2C_TLL_ADD 0x02
#define I2C_TLH_ADD 0x03
#define I2C_THL_ADD 0x04
#define I2C_THH_ADD 0x05
#define I2C_INT_ADD 0x06

/*Threshold bytes*/
#define TLL 0x0F
#define TLH 0x00
#define THL 0x00
#define THH 0x08

/*Slave register values*/
#define PERSIST 0x04
#define INTR_FIELD 0x01
#define POWER 0x03
#define TIMING 0x01      //Low gain
#define ADC0_DLOW 0x0C

#define ENABLE_ADD 0x80
#define CTRL_ADD 0x8F
#define PDATA_ADD 0x9C
#define STATUS_ADD 0x93
#define GCONF1_ADD 0xA2
#define GCONF3_ADD 0xAA
#define GCONF4_ADD 0xAB
//#define GCONF4_ADD 0xAC
#define GPENTH_ADD 0xA0
#define GEXTH_ADD 0xA1
#define GFLVL_ADD  0xAE
#define GSTATUS_ADD 0xAF
#define GFIFO_U_ADD 0xFC
#define GFIFO_D_ADD 0xFD
#define GFIFO_L_ADD 0xFE
#define GFIFO_R_ADD 0xFF

#define BM_SLAVE_ADD 0x76
#define BM_ID_ADD 0xD0
#define BM_RESET_ADD 0xE0
#define BM_CTRL_HUM_ADD 0xF2
#define BM_STATUS_ADD 0xF3
#define BM_CTRL_MEAS_ADD 0xF4
#define BM_CONFIG_ADD 0xF5
#define BM_PRESS_MSB_ADD 0xF7
#define BM_PRESS_LSB_ADD 0xF8
#define BM_PRESS_XLSB_ADD 0xF9
#define BM_TEMP_MSB_ADD 0xFA
#define BM_TEMP_LSB_ADD 0xFB
#define BM_TEMP_XLSB_ADD 0xFC
#define BM_HUM_MSB_ADD 0xFD
#define BM_HUM_LSB_ADD 0xFE

#define BM_DIG_H1_ADD 0xA1
#define BM_DIG_H2_MSB_ADD 0xE1
#define BM_DIG_H2_LSB_ADD 0xE2
#define BM_DIG_H3_ADD 0xE3
#define BM_DIG_H4_MSB_ADD 0xE4
#define BM_DIG_H4_LSB_ADD 0xE5
#define BM_DIG_H5_MSB_ADD 0xE5
#define BM_DIG_H5_LSB_ADD 0xE6
#define BM_DIG_H6_ADD 0xE7

#define BM_ID_VALUE 0x60
#define BM_RESET_VALUE 0xB6
#define BM_CTRL_HUM_VALUE 0x1
#define BM_CTRL_MEAS_VALUE 0x4B
#define BM_CONFIG_VALUE 0x28

//#define BM_DIG_T1_MSB_ADD

/*SLeep routine array*/
unsigned int sleep_block_counter[4];

/*Slave addresses and read/write operation values */
#define TSL_addr 0x39
#define Write 0x0
#define Read 0x1

/*Port and Pin COnstants*/
#define Clock_Data_Port gpioPortC
#define Power_Pin  0
#define Interrupt_Pin  1
#define Power_Interrupt_Port  gpioPortD

/*Miscellaneous constants */
#define CMD_Byte 0xC0
#define Word_mode 0xE0
#define INT_PIN_ADD 0x0002
#define SCL_Pin  5
#define SDA_Pin  4

/* ADC DMA Setup Constants*/
#define DMA_CHANNEL_ADC       0
#define ADCSAMPLES                        500
#define ADC_Prescale        1300000
volatile uint16_t ramBufferAdcData[ADCSAMPLES];//Buffer size
#define ADCSAMPLESPERSEC              100000
#define ADC0_resolution adcRes12Bit
#define ADC0_reference adcRef1V25
#define ADC0_channel adcSingleInpTemp
#define ADC0_acq_time adcAcqTime1
#define ADC0_rep true
#define ADC0_PRS_SELECTION adcPRSSELCh0

/* Energy mode enum*/
typedef enum{
EM0=0,
EM1=1,
EM2=2,
EM3=3
}sleepstate_enum;

char LE_mode;

DMA_CB_TypeDef cb;


uint32_t Sum;//store the sum of all the values of the buffer


int Samples_Count;

/*Initializing global variables*/
int   LEVEL=2;
float osc=1;
int Buffer_Count;
int LETIMER_Period=1;
int LED_state=0;
float avg,temp;

int32_t t_fine;
unsigned char dig_H1;
signed short dig_H2;
unsigned char dig_H3;
signed short dig_H4;
signed short dig_H5;
signed char dig_H6;

/* RTC nominal frequency */
#define RTC_FREQ               32768

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortB


//#if defined ( STK3700 )
  #define LED_PORT gpioPortE
  #define LED_PIN  2

uint32_t counter=0;
uint32_t counter1=0;
uint32_t button_counter=0;
static volatile uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static volatile uint16_t buttons_pressed;
static volatile uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static volatile uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static uint16_t channels_used_mask;
static uint8_t num_channels_used;
static float channel_threshold_percent[NUM_LESENSE_CHANNELS];
#endif /* MAIN_H_ */
