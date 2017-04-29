/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"
 * for details. Before using this software for any purpose, you must agree to the
 * terms of that agreement.
 *
 ******************************************************************************/

/*Assignment 4
 * Author: Sayan Barman
 *
 * Description : To develop a system where the Atmel ATSAMB11
 * is a master controller of a sensor hub, the Leopard Gecko.
 * The ATSAMB11 will take the status of Lightness and Darkness
 * to turn on its LED as well as take the temperature data
 * from the Leopard Gecko and transmit the temperature
 * through its Health Thermometer Service to the Atmel Smart application
 * on a smart phone or tablet.
 *
 *
 *   Disclaimer : I would like to give credit to SILICON LABS for the Sleep,
 *   blockSleepMode,unblockSleepMode,converttoCelcius routines*/


/*Including header files*/
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
#include "em_rtc.h"
#include "em_lesense.h"
#include "lesense_letouch.h"
#include "lesense_letouch_config.h"
#include "em_leuart.h"
#include "main.h"
#include "circular_Buffer.h"

#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30

/***options***/
//#define ENERGY_MODE EM2

#define CALIB_ON 1
#define I2C_0_ACMP_1 0
#define DMA_STATUS 1
/****** LETIMER constants*****/
#define LFXOFreq 32768
#define ULFRCOFreq 1000
#define LED_PORT    gpioPortE
#define LED_LIGHT_PIN     2
#define LED_TEMP_PIN     3
#define Period     5
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





/*SLeep routine array*/
unsigned int sleep_block_counter[4];

/*Slave addresses and read/write operation values */
#define TSL_addr 0x39
#define Write 0x0
#define Read 0x1

/*Port and Pin COnstants*/
#define Clock_Data_Port gpioPortC
#define Power_Pin_Gesture  0
#define Power_Pin_BME 3
#define Interrupt_Pin  1
#define Power_Interrupt_Port  gpioPortD

/*Miscellaneous constants */
#define SCL_Pin  5
#define SDA_Pin  4


/* Energy mode enum*/
#define ENERGY_MODE EM2


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


#define ENABLE_ADD 0x80
#define CTRL_ADD 0x8F
#define PDATA_ADD 0x9C
#define STATUS_ADD 0x93
#define GCONF1_ADD 0xA2
#define GCONF2_ADD 0xA3
#define GCONF3_ADD 0xAA
#define GCONF4_ADD 0xAB
//#define GCONF4_ADD 0xAC
#define GPENTH_ADD 0xA0
#define GEXTH_ADD 0xA1
#define CONFIG1_ADD        0x8D
//#define LDRIVE_ADD          LED_DRIVE_100MA
#define PILT_ADD            0x89       // Low proximity threshold
#define PIHT_ADD            0X8B      // High proximity threshold
//#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define CONFIG2_ADD         0x90    // No saturation interrupts or LED boost
#define CONFIG3_ADD         0X9F       // Enable all photodiodes, no SAI
//#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
//#define DEFAULT_GGAIN           GGAIN_4X
//#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
//#define DEFAULT_GWTIME          GWTIME_2_8MS
//#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define GPULSE_ADD          0xA6    // 32us, 10 pulses
//#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
//#define DEFAULT_GIEN            0       // Disable gesture interrupts

#define GFLVL_ADD  0xAE
#define GSTATUS_ADD 0xAF
#define GFIFO_U_ADD 0xFC
#define GFIFO_D_ADD 0xFD
#define GFIFO_L_ADD 0xFE
#define GFIFO_R_ADD 0xFF

//float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

/*SLeep routine array*/
unsigned int sleep_block_counter[4];

/*Slave addresses and read/write operation values */
#define TSL_addr 0x39
#define Write 0x0
#define Read 0x1

/*Port and Pin COnstants*/
#define Clock_Data_Port gpioPortC
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

#define GGAIN_4X                2
#define LED_DRIVE_100MA         0
#define GWTIME_2_8MS            1

#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30
#define DEFAULT_CONFIG1         0x0
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0      // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts




char LE_mode;

DMA_CB_TypeDef cb;


uint32_t Sum;//store the sum of all the values of the buffer

uint16_t channels_touched = 0;

int Samples_Count;
uint8_t swipe=0;

/*Initializing global variables*/
int Buffer_Count;
float avg,temp;


char LE_mode;
int Gesture_value;
double final_temp_val;
double final_hum_val;
uint8_t Hand_value=0;


void CMU_Setup();
void LETIMER0_Setup();
void GPIO_Setup();
void setupI2C(void);
static void LETOUCH_setupACMP(void);
static void LETOUCH_setupLESENSE(void);
static void LETOUCH_setupGPIO(void);
static void LETOUCH_setupRTC(void);
static void LETOUCH_setupCMU(void);

static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N);
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N);

void sleep();
void blockSleepMode(sleepstate_enum minimumMode);
void unblockSleepMode(sleepstate_enum minimumMode);


int LEUART_interrupt_count;


/* DMA callback structure */
DMA_CB_TypeDef cb;


uint32_t Sum;//store the sum of all the values of the buffer


int Samples_Count;//to count the number of samples in the buffer

void Clock_disable()
{
/* Enable clocks */
	CMU_ClockEnable(cmuClock_CORELE, false);
	CMU_ClockEnable(cmuClock_LETIMER0, false);
	//CMU_ClockEnable(cmuClock_DMA, false);
	//CMU_ClockEnable(cmuClock_ADC0, false);
	//CMU_ClockEnable(cmuClock_ACMP0, false);
	CMU_ClockEnable(cmuClock_I2C0, false);
	CMU_ClockEnable(cmuClock_I2C1, false);

	  /* Enabling clocks, all other remain disabled */
	  CMU_ClockEnable(cmuClock_GPIO, false);       /* Enable GPIO clock */
	  /* Start LFXO, and use LFXO for low-energy modules */
	         	//
	  //CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	  CMU_ClockEnable(cmuClock_LEUART0,false);    /* Enable LEUART1 clock */
     //if(Calibration==1)
    //{

       /* Enable clock for TIMER module */
       //CMU_ClockEnable(cmuClock_TIMER0, false);
       //CMU_ClockEnable(cmuClock_TIMER1, false);
       CMU_ClockEnable(cmuClock_HFPER, false);
       CMU_OscillatorEnable(cmuOsc_LFXO,false,false);
       CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);


    }

static void LETOUCH_setupCMU( void )
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  CMU_ClockEnable(cmuClock_ACMP1, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

 /* Low energy peripherals
 *   LESENSE
 *   LFXO clock must be enabled prior to enabling
 *   clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Disable clock source for LFB clock */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
}


void LETOUCH_Init(float sensitivity[]){

  uint8_t i;
  channels_used_mask = 0;
  num_channels_used = 0;

  /* Initialize channels used mask and threshold array for each channel */
  /* Uses the sensitivity array to deduce which channels to enable and how */
  /* many channels that are enabled */

  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){

    /* Init min and max values for each channel */
    channel_max_value[i] = 0;
    channel_min_value[i] = 0xffff;

    /* Add to channels used mask if sensitivity is not zero */
    if(sensitivity[i] != 0.0){
      channel_threshold_percent[i] = sensitivity[i];
      channels_used_mask |= (1 << i);
      num_channels_used++;
    }
  }

  /* Disable interrupts while initializing */
  INT_Disable();

  /* Setup CMU. */
  LETOUCH_setupCMU();
  /* Setup GPIO. */
  LETOUCH_setupGPIO();
  /* Setup ACMP. */
  LETOUCH_setupACMP();
  /* Setup LESENSE. */
  LETOUCH_setupLESENSE();
  /* Do initial calibration "N_calibration_values * 10" times to make sure */
  /* it settles on values after potential startup transients */
  for(i = 0; i < NUMBER_OF_CALIBRATION_VALUES * 10; i++){
    LESENSE_ScanStart();
    LETOUCH_Calibration();
  }
  /* Setup RTC for calibration interrupt */
  LETOUCH_setupRTC();
  /* Initialization done, enable interrupts globally. */
  INT_Enable();

}

/***************************************************************************//**
 * @brief
 *   Get the buttons pressed variable, one bit for each channel pressed
 *   or'ed together.
 *
 * @return
 *   The touch buttons/pads that are in touched state is or'ed together and
 *   returned.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelsTouched(void){

  return buttons_pressed;
}

/***************************************************************************//**
 * @brief
 *   Get the maximum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get maximum value for
 *
 * @return
 *   The maximum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel){

  return channel_max_value[channel];
}

/***************************************************************************//**
 * @brief
 *   Get the minimum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get minimum value for
 *
 * @return
 *   The minimum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel){

  return channel_min_value[channel];
}

static void LETOUCH_setupACMP( void )
{
  /* Configuration structure for ACMP */
  /* See application note document for description of the different settings. */
  static const ACMP_CapsenseInit_TypeDef acmpInit =
  {
    .fullBias                 = true,            //Configured according to application note
    .halfBias                 = true,            //Configured according to application note
    .biasProg                 = 0x5,             //Configured according to application note
    .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
    .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
    .resistor                 = acmpResistor0,   //Configured according to application note
    .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
    .vddLevel                 = 0x30,            //Configured according to application note
    .enable                   = false            //LESENSE enables the ACMP
  };

  /* Initialize ACMP in capsense mode*/
  ACMP_CapsenseInit(ACMP0, &acmpInit);
  ACMP_CapsenseInit(ACMP1, &acmpInit);

}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
static void LETOUCH_setupLESENSE( void )
{
  uint8_t i;

  /* LESENSE configuration structure */
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    {
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle,
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0x0
    },

    .perCtrl          =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = false,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = false,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = true,
    .chPinExMode   = lesenseChPinExDis,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkLF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x0,
    .sampleDelay   = SAMPLE_DELAY,
    .measDelay     = 0x0,
    .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntLevel,
    .cntThres      = 0x0,                   // Configured later by calibration function
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channels */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      LESENSE_ChannelConfig(&initLesenseCh, i);
    }
  }

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LESENSE_SCAN_FREQUENCY);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
static void LETOUCH_setupGPIO( void )
{
  unsigned int i;

  /* Set GPIO pin mode to disabled for all active pins */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      GPIO_PinModeSet(LESENSE_CH_PORT, i, gpioModeDisabled, 0);
    }
  }
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void LETOUCH_setupRTC( void )
{
  /* RTC configuration */
  static const RTC_Init_TypeDef rtcInit =
  {
    .enable   = true,
    .debugRun = false,
    .comp0Top = true
  };

  RTC_Init(&rtcInit);

  /* Set the RTC calibration interrupt compare value */
  /* calibration interval defined in lesense_letouch_config.h */
  RTC_CompareSet( 0, CALIBRATION_INTERVAL * RTC_FREQ );

  RTC_IntEnable(RTC_IFS_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * Calibration function
*****************************************************************************/
void LETOUCH_Calibration( void ){
  int i,k;
  uint16_t nominal_count;
  static uint8_t calibration_value_index = 0;

  /* Wait for current scan to finish */
  while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

  /* Get position for first channel data in count buffer from lesense write pointer */
  k = ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);

  /* Handle circular buffer wraparound */
  if(k >= num_channels_used){
    k = k - num_channels_used;
  }
  else{
    k = k - num_channels_used + NUM_LESENSE_CHANNELS;
  }

  /* Fill calibration values array with buffer values */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      calibration_value[i][calibration_value_index] = LESENSE_ScanResultDataBufferGet(k++);
    }
  }

  /* Wrap around calibration_values_index */
  calibration_value_index++;
  if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES){
    calibration_value_index = 0;
  }

  /* Calculate max/min-value for each channel and set threshold */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      channel_max_value[i] = GetMaxValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);
      channel_min_value[i] = GetMinValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);

      nominal_count = channel_max_value[i];
      LESENSE_ChannelThresSet(i, 0x0,(uint16_t) (nominal_count - ((nominal_count * channel_threshold_percent[i])/100.0)) );
    }
  }
}

/**************************************************************************//**
 * Insertion sort, can be used to implement median filter instead of just using max-value
 * in calibration function.
*****************************************************************************/
/*
static void InsertionSort(uint16_t* A, uint16_t N){
  int i,j;
  uint16_t temp;

  for(i=1; i<N; i++)
  {
    temp = A[i];
    j = i-1;
    while(temp<A[j] && j>=0)
    {
      A[j+1] = A[j];
      j = j-1;
    }
    A[j+1] = temp;
  }
}
*/

/**************************************************************************//**
 * Returns maximum value in input array of size N
*****************************************************************************/
static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t max = 0;

  for(i=0; i<N; i++)
  {
    if(max < A[i]){
      max = A[i];
    }
  }
  return max;
}

/**************************************************************************//**
 * Returns minimum value in input array of size N
*****************************************************************************/
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t min = 0xffff;

  for(i=0; i<N; i++)
  {
    if(A[i] < min){
      min = A[i];
    }
  }
  return min;
}

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC, used for the calibration function
 *****************************************************************************/
void RTC_IRQHandler( void )
{
  /* Clear interrupt flag */
  RTC_IntClear(RTC_IFS_COMP0);

  LETOUCH_Calibration();

  /* Reset counter */
  RTC_CounterReset();
}

/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler( void )
{
  uint8_t channel, i, valid_touch;
  uint32_t interrupt_flags, tmp, channels_enabled;
  uint16_t threshold_value;

  /* Get interrupt flag */
  interrupt_flags = LESENSE_IntGet();
  /* Clear interrupt flag */
  LESENSE_IntClear(interrupt_flags);

  /* Interrupt handles only one channel at a time */
  /* therefore only first active channel found is handled by the interrupt. */
  for(channel = 0; channel < NUM_LESENSE_CHANNELS; channel++){
    if( (interrupt_flags >> channel) & 0x1 ){
      break;
    }
  }

  /* To filter out possible false touches, the suspected channel is measured several times */
  /* All samples should be below threshold to trigger an actual touch. */

  /* Disable other channels. */
  channels_enabled = LESENSE->CHEN;
  LESENSE->CHEN = 1 << channel;

   /* Evaluate VALIDATE_CNT results for touched channel. */
  valid_touch = 1;

  for(i = 0;i<VALIDATE_CNT;i++){
    /* Start new scan and wait while active. */
    LESENSE_ScanStart();
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

    tmp = LESENSE->SCANRES;
    if((tmp & (1 << channel)) == 0){
      valid_touch = 0;
    }
  }

  /* Enable all channels again. */

  LESENSE->CHEN = channels_enabled;


  if(valid_touch){
    /* If logic was switched clear button flag and set logic back, else set button flag and invert logic. */
    if(LESENSE->CH[channel].EVAL & LESENSE_CH_EVAL_COMP){
      buttons_pressed &= ~(buttons_pressed);
      LESENSE->CH[channel].EVAL &= ~LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value -= 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
    else{
      buttons_pressed |= (1 << channel);
      //if(button_counter==1)
      if(buttons_pressed)
      {
      LESENSE->CH[channel].EVAL |= LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value += 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
      button_counter=0;
      }
      //else
      //button_counter++;
    }

  }

  /* Need to reset RTC counter so we don't get new calibration event right after buttons are pushed/released. */
  RTC_CounterReset();

}


void circ_buff(void)
{
	//int i;
_transmit_buffer.buff = allocate_memory(&_transmit_buffer);
			_transmit_buffer.head=_transmit_buffer.buff;
					 	_transmit_buffer.tail=_transmit_buffer.buff;
					 	_transmit_buffer.buff=_transmit_buffer.buff;
					 	_transmit_buffer.num_items= 0;
					 	_transmit_buffer.length = MAX_LEN;
}


LEUART_Init_TypeDef leuart0Init =
{
  .enable   = leuartEnableTx,       /* Activate data reception on LEUn_TX pin. */
  .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = 9600,                 /* Baudrate = 9600 bps */
  .databits = leuartDatabits8,      /* Each LEUART frame contains 8 databits */
  .parity   = leuartNoParity,       /* No parity bits in use */
  .stopbits = leuartStopbits1,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};


void initLeuart(void)
{
  /* Reseting and initializing LEUART1 */
  LEUART_Reset(LEUART0);
  	    CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFRCO);
  	  //CMU_ClockEnable(cmuClock_LEUART0,true);
  LEUART_Init(LEUART0, &leuart0Init);

  /* Route LEUART1 TX pin to DMA location 0 */
  LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. TX is on C6 */
  GPIO_PinModeSet(Transmit_Port,                /* GPIO port */
		  	  	  Transmit_Pin,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */


  LEUART0->CMD |=(LEUART_CMD_TXEN | LEUART_CMD_RXEN);//Enabling Receiver and Transmitter
  LEUART0->CTRL |=(LEUART_CTRL_LOOPBK | LEUART_CTRL_AUTOTRI);//Enabling Autotri state and loopback
  //LEUART0->IEN |=LEUART_IEN_TXBL;
  NVIC_EnableIRQ(LEUART0_IRQn);//Enabling interrupt handler


}



/*Function to convert adc Sample to Celcius taken from Bosch*/
double BME280_compensate_T_double(int32_t adc_T)
{
double var1, var2, T;
var1 = (((double)adc_T)/16384.0 - ((double)27504)/1024.0) * ((double)26435);
var2 = ((((double)adc_T)/131072.0 - ((double)27504)/8192.0) *
(((double)adc_T)/131072.0 - ((double) 27504)/8192.0)) * (-((double)(1000)));
t_fine = (int32_t)(var1 + var2);
T = (var1 + var2) / 5120.0;
return T;
}



/*Function to convert adc Sample to %humidity taken from Bosch*/
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
int32_t v_x1_u32r;
v_x1_u32r = (t_fine - ((int32_t)76800));
v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
((int32_t)dig_H2) + 8192) >> 14));
v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
//v_x1_u32r = (v_x1_u32r > (419430400) ? (419430400) : v_x1_u32r);
return (uint32_t)(v_x1_u32r>>12);
}





/****Clock Management Unit Setup*****/
void CMU_Setup()
{
	SystemCoreClockUpdate();


	    CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
	    CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFXO);
	    CMU_ClockEnable(cmuClock_LEUART0,true);
	    CMU_ClockEnable(cmuClock_LETIMER0, false);
	    CMU_ClockEnable(cmuClock_GPIO,true);


	    	    CMU_ClockEnable(cmuClock_CORELE, true);
	    	    CMU_ClockEnable(cmuClock_HFPER,true);
	    		CMU_ClockEnable(cmuClock_LEUART0,true);
	    		CMU_ClockEnable(cmuClock_I2C0, true);
	    	    CMU_ClockEnable(cmuClock_I2C1, true);


}

void LETIMER0_Setup(void)
{
	float z;
	unsigned int Comp0;
	unsigned int Comp1;
	unsigned int LETIMER0_prescaler;
	if(ENERGY_MODE==EM3)
		Comp0 = Period*ULFRCOFreq*osc;
	else
	{
		LETIMER0_prescaler= Period/2;
		CMU->LFAPRESC0&= 0xfffff0ff;//clearing the prescaler register
		CMU->LFAPRESC0|= LETIMER0_prescaler << 8;//shift prescaler into position
		LETIMER0_prescaler = 1 << LETIMER0_prescaler ;//the value of prescaler register
		Comp0=Period*(LFXOFreq/LETIMER0_prescaler);// moving the final value to Comp0

	}
	if(ENERGY_MODE==EM3)
	{
		z = ON_DUTY_CYCLE*ULFRCOFreq*osc;

	if(z > (int)z)
	{
		Comp1=z+1;
	}
	}
		else
		{
			Comp1=ON_DUTY_CYCLE*(LFXOFreq/LETIMER0_prescaler);

		}

    /* Set initial compare values for COMP0 and Comp1 */
    LETIMER_CompareSet(LETIMER0, 0,Comp0);
    LETIMER_CompareSet(LETIMER0,1,Comp1);
    /* Set configurations for LETIMER 0 */
    const LETIMER_Init_TypeDef letimerInit =
    {
        .enable         = true,                   /* start counting when init completed - only with RTC compare match */
        .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
        .rtcComp0Enable = false,                  /* DON'T Start counting on RTC COMP0 match. */
        .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
        .comp0Top       = true,                  /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
        .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
        .out0Pol        = 0,                      /* Idle value for output 0. */
        .out1Pol        = 0,                      /* Idle value for output 1. */
        .ufoa0          = letimerUFOANone,        /* Pulse output on output 0 */
        .ufoa1          = letimerUFOANone,        /* No output on output 1*/
        .repMode        = letimerRepeatFree,      /*Free mode*/
    };

    /* Initialize LETIMER */
    CMU_ClockEnable(cmuClock_LETIMER0, true);
    LETIMER_Init(LETIMER0, &letimerInit);
   /*****enabling interrupts for COmP1 and UF***/
    LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP1);
    //LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
    blockSleepMode(ENERGY_MODE);
    NVIC_EnableIRQ(LETIMER0_IRQn);//enabling interrupt table

}
/****GPIO setup***/
void GPIO_Setup()
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	//GPIO_PinModeSet(LED_PORT, LED_LIGHT_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LED_PORT, LED_TEMP_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(Transmit_Port, Transmit_Pin, gpioModePushPull, 0);
	GPIO_PinModeSet(Power_Interrupt_Port,Power_Pin_BME, gpioModePushPull,0);

	GPIO_PinModeSet(Power_Interrupt_Port,Power_Pin_Gesture, gpioModePushPull,0);

}

void I2C_READ2(void)
{
		INT_Disable();


	      int i=0;
	      uint32_t arr[8];
	      I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Write);
	      I2C1->CMD =I2C_CMD_START;
	      if(buttons_pressed==0)
	      {
	      while(!(I2C1->IF & I2C_IF_ACK));
	      I2C1->IFC = I2C_IFC_ACK;


	      I2C1->TXDATA= BM_PRESS_MSB_ADD;
	      while(!(I2C1->IF & I2C_IF_ACK));
	      I2C1->IFC |= I2C_IFC_ACK;

	      I2C1->CMD =I2C_CMD_START;
	      I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Read);
	      while(!(I2C1->IF & I2C_IF_ACK));
	      I2C1->IFC = I2C_IFC_ACK;

	      for(i=0;i<8;i++)
	      {
	      while(!(I2C1->IF & I2C_IF_RXDATAV));
	       arr[i] = I2C1->RXDATA;
	      if(i==7)
	    	  I2C1->CMD =I2C_CMD_NACK;
	      else
	       I2C1->CMD =I2C_CMD_ACK;

	      }


	      I2C1->CMD = I2C_CMD_STOP;
	      while((I2C1->IF & I2C_IF_MSTOP)==0);
	      I2C1->IFC = I2C_IFC_MSTOP;

	      uint32_t last=(arr[5]>>4);


	      uint32_t adc_temp_val=((arr[3]<<12)|(arr[4])<<4|(last));
	      final_temp_val=(double)(BME280_compensate_T_double(adc_temp_val)-7);
	      int avg1=(int8_t)final_temp_val;
	      	float diff= (final_temp_val - avg1)*10;
	      put_item_tobuffer(&_transmit_buffer,'T');
	      put_item_tobuffer(&_transmit_buffer,avg1);
	      put_item_tobuffer(&_transmit_buffer,(int8_t)diff);
	      uint32_t adc_hum_val=((arr[6]<<8) | (arr[7]));

	      final_hum_val=(double)((double)(bme280_compensate_H_int32(adc_hum_val)))/(double)2048;

	      int avg2=(int8_t)final_hum_val;
	      float diff1=(final_hum_val - avg2)*10;
	      put_item_tobuffer(&_transmit_buffer,'H');
	      put_item_tobuffer(&_transmit_buffer,avg2);
	      put_item_tobuffer(&_transmit_buffer,(int8_t)diff1);
	      put_item_tobuffer(&_transmit_buffer,'G');
	      put_item_tobuffer(&_transmit_buffer,swipe);
	      }
	      else
	    	  return;
	   	    INT_Enable();
   //int z;
	}




/****LETMER Handler****/
void LETIMER0_IRQHandler(void)
{
	INT_Disable();
	if(buttons_pressed!=0)
		return;
	int intFlags;
	intFlags = LETIMER0->IF;//the value of interrupt flag
	if((intFlags & LETIMER_IF_COMP1)!=0)                            //checking for COMP1 Interrupt
	{
		LETIMER0->IFC=intFlags;//clearing Interrupts
		if(buttons_pressed==0)
		{
		I2C_READ2();
		LEUART0->IEN |=LEUART_IEN_TXBL;
		}

	}

INT_Enable();
}



/*Sleep routine taken from SILICON LABS*/
void sleep(void)
{
	if (sleep_block_counter[0] > 0)
	{
		return; // Blocked everything below EM0, so just return
	}
	else if(sleep_block_counter[1] > 0)
	{
	    EMU_EnterEM1(); // Blocked everything below EM1, enter EM1

	}
	else if(sleep_block_counter[2] > 0)
	{
		EMU_EnterEM2(true); //Blocked everything below EM2, enter EM2

	}
	else if(sleep_block_counter[3]>0)
	{
		EMU_EnterEM3(true); //Blocked everything below EM3, enter EM3

	}
	else
    return;
}


/***Block Sleep Mode routine taken from SILICON LABS**/
void blockSleepMode(sleepstate_enum minimumMode)
{
	INT_Disable();
	sleep_block_counter[minimumMode]++;
	INT_Enable();
}

/*UNblock Sleep Routine taken from SILICON LABS**/
void unblockSleepMode(sleepstate_enum minimumMode)
{
	INT_Disable();
	if(sleep_block_counter[minimumMode]>0)
	sleep_block_counter[minimumMode]--;
	else
		sleep_block_counter[minimumMode]=0;
	INT_Enable();
}

void I2C_READ1(void)
{
	struct structure
	{
		unsigned char dig_A1;
		signed short dig_E1;
		signed short dig_E2;
		unsigned char dig_E3;
		signed short dig_E4;
		signed short dig_E5;
		signed short dig_E6;
		signed char dig_E7;
	}humidity;

	uint32_t arr1[7];
	int i;

	I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Write);
    I2C1->CMD =I2C_CMD_START;
    while(!(I2C1->IF & I2C_IF_ACK));
	I2C1->IFC = I2C_IFC_ACK;


		      I2C1->TXDATA= BM_DIG_H1_ADD;
		      while(!(I2C1->IF & I2C_IF_ACK));
		      I2C1->IFC |= I2C_IFC_ACK;

		      I2C1->CMD =I2C_CMD_START;
		      I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Read);
		      while(!(I2C1->IF & I2C_IF_ACK));
		      I2C1->IFC = I2C_IFC_ACK;

		      while(!(I2C1->IF & I2C_IF_RXDATAV));
		      humidity.dig_A1 = I2C1->RXDATA;
		      I2C1->CMD =I2C_CMD_ACK;

		      I2C1->CMD = I2C_CMD_STOP;
		      while((I2C1->IF & I2C_IF_MSTOP)==0);
		      I2C1->IFC = I2C_IFC_MSTOP;

		      I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Write);
		      I2C1->CMD =I2C_CMD_START;
		      while(!(I2C1->IF & I2C_IF_ACK));
		      I2C1->IFC = I2C_IFC_ACK;


		      		      I2C1->TXDATA= BM_DIG_H2_MSB_ADD;
		      		      while(!(I2C1->IF & I2C_IF_ACK));
		      		      I2C1->IFC |= I2C_IFC_ACK;

		      		      I2C1->CMD =I2C_CMD_START;
		      		      I2C1->TXDATA=((BM_SLAVE_ADD<<1)|Read);
		      		      while(!(I2C1->IF & I2C_IF_ACK));
		      		      I2C1->IFC = I2C_IFC_ACK;



		      		    for(i=0;i<6;i++)
		      		    {
		      		    	while(!(I2C1->IF & I2C_IF_RXDATAV));
		      		    			       arr1[i]= I2C1->RXDATA;
		      		    			      I2C1->CMD =I2C_CMD_ACK;
		      		    }

		      		  while(!(I2C1->IF & I2C_IF_RXDATAV));
		      		  		       arr1[i] = I2C1->RXDATA;
		      		  		      I2C1->CMD =I2C_CMD_NACK;
		      		  humidity.dig_E1 = arr1[0];
		      		  humidity.dig_E2 = arr1[1];
		      		  humidity.dig_E3 = arr1[2];
		      		  humidity.dig_E4 = arr1[3];
		      		  humidity.dig_E5 = arr1[4];
		      		  humidity.dig_E6 = arr1[5];
		      		  humidity.dig_E7 = arr1[6];


		      dig_H1=(humidity.dig_A1 & 0xFF);
		      dig_H2=((((humidity.dig_E1) & 0xFF) <<8)|(((humidity.dig_E2) & 0xFF00)>>8));
		      dig_H3=(humidity.dig_E3 & 0xFF);
		      dig_H4=(((humidity.dig_E4) & 0xFF0)|((humidity.dig_E5)& 0xF));
		      dig_H5=((((humidity.dig_E5) & 0xF0)<<4) | (((humidity.dig_E6) & 0xFF0)>>4));
		      dig_H6=(humidity.dig_E7);
		      //int q;



}





void i2c_one_Transfer(uint8_t address,uint8_t data)
{
	uint8_t add = address;
	    uint8_t value = data;


		I2C1->TXDATA=(BM_SLAVE_ADD<<1)|Write;   //Write operation
		for(int i=0;i<500;i++);
		I2C1->CMD = I2C_CMD_START;
		while(!(I2C1->IF & I2C_IF_ACK));
		I2C1->IFC = I2C_IFC_ACK;

		I2C1->TXDATA = (add);     //Sending address of the register
		while(!(I2C1->IF & I2C_IF_ACK));
	    I2C1->IFC = I2C_IFC_ACK;

	    I2C1->TXDATA = value;              //Sending value into the register
	    while(!(I2C1->IF & I2C_IF_ACK));
	    I2C1->IFC = I2C_IFC_ACK;

	    I2C1->CMD = I2C_CMD_STOP;
	    while((I2C1->IF & I2C_IF_MSTOP)==0);
	    I2C1->IFC = I2C_IFC_MSTOP;

	 //   }
}

void i2c_zero_Transfer(uint8_t address,uint8_t data)
{
	uint8_t add = address;
	    uint8_t value = data;


		I2C0->TXDATA=(TSL_addr<<1)|Write;   //Write operation
		for(int i=0;i<500;i++);
		I2C0->CMD = I2C_CMD_START;
		while(!(I2C0->IF & I2C_IF_ACK));
		I2C0->IFC = I2C_IFC_ACK;

		I2C0->TXDATA = (add);     //Sending address of the register
		while(!(I2C0->IF & I2C_IF_ACK));
	    I2C0->IFC = I2C_IFC_ACK;

	    I2C0->TXDATA = value;              //Sending value into the register
	    while(!(I2C0->IF & I2C_IF_ACK));
	    I2C0->IFC = I2C_IFC_ACK;

	    I2C0->CMD = I2C_CMD_STOP;
	    while((I2C0->IF & I2C_IF_MSTOP)==0);
	    I2C0->IFC = I2C_IFC_MSTOP;

	 //   }
}

void Slave_Setup()
{

	GPIO_PinOutSet(Power_Interrupt_Port,Power_Pin_Gesture);//Providing Power to the slave
	GPIO_PinOutSet(Power_Interrupt_Port,Power_Pin_BME);//Providing Power to the slave
	for(int i=0;i<500;i++);

	i2c_zero_Transfer(ENABLE_ADD,0x00);
	i2c_zero_Transfer(GCONF1_ADD,DEFAULT_GCONF1);
	i2c_zero_Transfer(GCONF2_ADD,DEFAULT_GGAIN);
	i2c_zero_Transfer(GCONF2_ADD,DEFAULT_GWTIME);
	i2c_zero_Transfer(GCONF2_ADD,DEFAULT_GLDRIVE);
	i2c_zero_Transfer(GCONF3_ADD,DEFAULT_GCONF3);
	i2c_zero_Transfer(GPENTH_ADD,DEFAULT_GPENTH);
	i2c_zero_Transfer(GEXTH_ADD,DEFAULT_GEXTH);
	i2c_zero_Transfer(CONFIG2_ADD,DEFAULT_CONFIG2);
	i2c_zero_Transfer(CONFIG3_ADD,DEFAULT_CONFIG3);
	i2c_zero_Transfer(GCONF4_ADD,0x2); //Higher Threshold Lower Byte
	i2c_zero_Transfer(ENABLE_ADD,0x45);

	i2c_one_Transfer(BM_RESET_ADD,BM_RESET_VALUE);    //Control Register
	i2c_one_Transfer(BM_CTRL_MEAS_ADD,BM_CTRL_MEAS_VALUE);      //Lower Threshold Higher Byte
	i2c_one_Transfer(BM_CTRL_HUM_ADD,BM_CTRL_HUM_VALUE);//Higher Threshold Lower Byte


	 GPIO_PinModeSet(Power_Interrupt_Port, Interrupt_Pin , gpioModeInput, 1);//Interrupt Pin is setup
     GPIO_ExtIntConfig(Power_Interrupt_Port, Interrupt_Pin , 1,false, true, true);//External Interrupt is enabled
	 NVIC_EnableIRQ(GPIO_ODD_IRQn);


}

/**Routine to setup the I2C bus**/
void setupI2C(void)
{
	I2C_Init_TypeDef i2cInit =
		{
	     .enable   = true,
		 .master   = true,
		 .refFreq  = 0,
		 .freq     = I2C_FREQ_STANDARD_MAX,
		 .clhr     = i2cClockHLRStandard,
		};
	I2C_Init_TypeDef i2cInit1 =
		{
	     .enable   = true,
		 .master   = true,
		 .refFreq  = 0,
		 .freq     = I2C_FREQ_STANDARD_MAX,
		 .clhr     = i2cClockHLRStandard,
		};
	GPIO_PinModeSet(Clock_Data_Port, SCL_Pin, gpioModeWiredAndPullUpFilter, 1);
	  GPIO_PinModeSet(Clock_Data_Port, SDA_Pin, gpioModeWiredAndPullUpFilter, 1);
	  /* Using PD6 (SDA) and PD7 (SCL) */
	    GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1);
	    GPIO_PinModeSet(gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1);

	  I2C0->ROUTE = I2C_ROUTE_SDAPEN |
	                  I2C_ROUTE_SCLPEN |
	                  (1 << _I2C_ROUTE_LOCATION_SHIFT);

	  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
	                  I2C_ROUTE_SCLPEN |
	                  (0 << _I2C_ROUTE_LOCATION_SHIFT);
	  I2C_Init(I2C0, &i2cInit);
	  I2C_Init(I2C1, &i2cInit1);
	  if(I2C0->STATE & I2C_STATE_BUSY)
	  	  {
	  		  I2C0->CMD=I2C_CMD_ABORT;
	  	  }
	  if(I2C1->STATE & I2C_STATE_BUSY)
	  {
		  I2C1->CMD=I2C_CMD_ABORT;
	  }
	  for (int i = 9; i > 0; i--)           //flushing out the previous data
	    {

	      GPIO_PinModeSet(Clock_Data_Port, SCL_Pin, gpioModeWiredAnd, 0);
	      GPIO_PinModeSet(Clock_Data_Port, SCL_Pin, gpioModeWiredAnd, 1);
	    }

	  for (int i = 9; i > 0; i--)           //flushing out the previous data
	  	    {

	  	      GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAnd, 0);
	  	      GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAnd, 1);
	  	    }

	for(int i=0;i<50;i++);
}

/*LEUART Handler*/
void LEUART0_IRQHandler()
{
	INT_Disable();
	int i=0;
	int j=0;
	if(ENERGY_MODE==EM3)
		CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFRCO);


	LEUART0->IEN &=~LEUART_IEN_TXBL;

	if(LEUART0->IF & LEUART_IF_TXBL){
/*'T' mode for temperature value,1 byte for the integer value and the other for the decimal part*/

		for(j=0;j<9;j++){
			LEUART0->TXDATA=(int32_t)read_delete_item_frombuffer(&_transmit_buffer);
					while((LEUART0->IF & LEUART_IF_TXC) ==0);
					initLeuart();
					for(i=0;i<5000;i++);

					LEUART0->IEN &=~LEUART_IEN_TXBL;
		}



		}


	INT_Enable();

}

void GPIO_ODD_IRQHandler(void)
{
	INT_Disable();
	int i=0,j,k;
	uint8_t arr[60]={0};
	uint8_t	arr1[131]={0};
	uint8_t arr2[32]={0};
	uint8_t arr3[32]={0};
	uint8_t arr4[32]={0};
	uint8_t arr5[32]={0};

      GPIO_IntClear(INT_PIN_ADD);
      //i2c_Transfer(GCONF3_ADD,0x3);


//                        for(j=0;j<arr[46];j++)
//                        {
                        I2C0->TXDATA=((TSL_addr<<1)|Write);
                              I2C0->CMD =I2C_CMD_START;
                              I2C0->IFC=I2C_IF_START;
                              while(!(I2C0->IF & I2C_IF_ACK));
                              I2C0->IFC = I2C_IFC_ACK;


                              I2C0->TXDATA= GFIFO_U_ADD;
                                    while(!(I2C0->IF & I2C_IF_ACK));
                                    I2C0->IFC |= I2C_IFC_ACK;

                                    I2C0->CMD =I2C_CMD_START;
                                    I2C0->IFC=I2C_IF_START;
                                    I2C0->TXDATA=((TSL_addr<<1)|Read);
                                    while(!(I2C0->IF & I2C_IF_ACK));
                                    I2C0->IFC = I2C_IFC_ACK;

                                    for(i=0;i<127;i++)
                                    {
                                    while(!(I2C0->IF & I2C_IF_RXDATAV));
                                    arr1[i] = I2C0->RXDATA;
                                    I2C0->CMD =I2C_CMD_ACK;
                                    }
                                    while(!(I2C0->IF & I2C_IF_RXDATAV));
                                    arr1[i] = I2C0->RXDATA;
                                    I2C0->CMD =I2C_CMD_NACK;

                                    I2C0->CMD = I2C_CMD_STOP;
                                    while((I2C0->IF & I2C_IF_MSTOP)==0);
                                    I2C0->IFC = I2C_IFC_MSTOP;

          if(Hand_value==0)
          {
          if(arr1[2]>arr1[0])
          {
        	  if(arr1[2]>arr1[1])
        	  {
        		  if(arr1[2]>arr1[3])
        		  {
        			  swipe++;
        		  }
        	  }
          }
          }
          else
          {
        	  if(Hand_value==1)
        	  {
        		  if(arr1[3]>arr1[0])
        		            {
        		          	  if(arr1[3]>arr1[1])
        		          	  {
        		          		  if(arr1[3]>arr1[2])
        		          		  {
        		          			  swipe++;
        		          		  }
        		          	  }
        		            }
        	  }
        	  }
          if(swipe==3)
          {
        	  GPIO_PinOutToggle(LED_PORT, LED_TEMP_PIN);
        	  swipe=0;
          }
   	    INT_Enable();

}




/****main****/
int main(void)
{

    /* Align different chip revisions */
     CHIP_Init();

      channels_touched = 0;

     float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

     /* Init GPIO for LED, turn LED off */
       CMU_ClockEnable(cmuClock_GPIO, true);
       GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

       /* Init Capacitive touch for channels configured in sensitivity array */
       LETOUCH_Init(sensitivity);


       while(LETOUCH_GetChannelsTouched() != 0);

    while(1)
      {


    	channels_touched = LETOUCH_GetChannelsTouched();
    			if((channels_touched==0x100) && counter ==0){
    			      /* Turn on LED */
    				GPIO_PinOutToggle(LED_PORT, LED_PIN);
    				counter++;
    				channels_touched=0;
    				Hand_value=0;
    			    //buttons_pressed=0;
    			     CMU_Setup();
    			    GPIO_Setup();
    			    circ_buff();
    			    initLeuart();
    			    setupI2C();

    			     Slave_Setup();
    			     I2C_READ1();
    			     LETIMER0_Setup();
    			     EMU_EnterEM2(true);


    			    }
    			else if((channels_touched==0x100) && counter ==1){
    			      /* Turn on LED */
    				//GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);
    				GPIO_PinOutToggle(LED_PORT, LED_PIN);
    				buttons_pressed &= ~(buttons_pressed);
    				channels_touched=0;
    				counter=0;
    				//Hand_value=1;
    					Clock_disable();


    				               	       /* All four slider pads enabled, the position in the array indicates which pin from PC0 to PC15. */

    				               	       /* Four pins active, PC8, PC9, PC10 and PC11. A value of 0.0 indicates inactive channel. */
    				               	         float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    				               	       /* Init GPIO for LED, turn LED off */
    				               	       CMU_ClockEnable(cmuClock_GPIO, true);
    				               	   LETOUCH_Init(sensitivity);
    				                  /* If any channels are touched while starting, the calibration will not be correct. */
    				                      /* Wait while channels are touched to be sure we continue in a calibrated state. */
    				                      while(LETOUCH_GetChannelsTouched() != 0);
    			}
    			else if((channels_touched==0x800) && counter1==0)
    			{
    				GPIO_PinOutToggle(LED_PORT, LED_PIN);
    				Hand_value=1;
    				counter1++;
    			}
    			else if((channels_touched==0x800) && counter1==0)
    			    			{
    			    				GPIO_PinOutToggle(LED_PORT, LED_PIN);
    			    				Hand_value=0;
    			    				counter1=0;
    			    			}

    	EMU_EnterEM2(true);

        	//sleep();

      }
}
