/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
//#include "boards.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BSP_LED_0      					NRF_GPIO_PIN_MAP(0, 13)
#define BSP_LED_1      					NRF_GPIO_PIN_MAP(0, 14)
#define BSP_LED_2      					NRF_GPIO_PIN_MAP(0, 15)
#define BSP_LED_3      					NRF_GPIO_PIN_MAP(0, 16)



#define IR_LED_PIN							NRF_GPIO_PIN_MAP(0, 4)
#define OBJ_DETECTION_CNT_MIN		10	// At least 10 consequtive positive readings

#define AD_IR_REFLECTION_VAL		170	// In ADC samples - false positive due to reflection from the IR glass. Measured experimentally.
#define AD_OBJ_DET_TRESHOLD			(AD_IR_REFLECTION_VAL + 10)	// In ADC samples (1023 max). Should be small number for more sensitivity, but not too small to detect noise.

#define SAMPLES_IN_BUFFER 10
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];	// Double buffered internal ADC data 
//static uint32_t							 m_ir_sensor_data_points_count = 0;
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
static volatile uint64_t 		 m_ulMicros = 0;
static volatile bool				 m_bIrPulse = false;

static volatile int32_t			 m_nAdSamplesPair[2];	// Contains averaged AD readings when IR was OFF and ON
static volatile float			 	 m_fAdVoltagePair[2];	// Contains averaged AD readings when IR was OFF and ON
static volatile uint32_t 		 m_unObjDetectionCounter = 0;

typedef enum {
	STATE_IR_OFF 	= 0,
	STATE_IR_ON		= 1
}EStates_t;

static volatile EStates_t		 m_eState = STATE_IR_OFF;

void onAdcDone(nrf_saadc_value_t* sAdBuff);
void onboardLedWrite(uint32_t unBspLed, bool bOn);
void irDiodeInit(void);
void irLedWrite(bool bOn);
bool compareAdcPairs(int32_t* pnBuff);

void onAdcDone(nrf_saadc_value_t* sAdBuff)
{
		// 10 samples collected - take average
		//NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);
		
		int32_t nAdcSum = 0;
		int32_t nAdcAverage = 0;
		
		// Ignore first sample
		for(int i=1; i<SAMPLES_IN_BUFFER; i++) {
			NRF_LOG_INFO("%d", sAdBuff[i])
			nAdcSum += sAdBuff[i];
		}
		
		nAdcAverage = (int32_t)(nAdcSum/(float)(SAMPLES_IN_BUFFER-1));
		

		if(m_eState == STATE_IR_OFF) {
			
			m_nAdSamplesPair[0] = nAdcAverage;
			m_fAdVoltagePair[0] = (float)nAdcAverage * (3.0/1023.0);	// 3.0V max, gain x5; 1023 max
			
			NRF_LOG_INFO("nAdcAverage = %d", m_nAdSamplesPair[0]);
			NRF_LOG_INFO("nAdcVoltageAverage = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_fAdVoltagePair[0]));
			
			// Change to next state
			m_eState = STATE_IR_ON;
		
		}
		else if(m_eState == STATE_IR_ON) {
			
			m_nAdSamplesPair[1] = nAdcAverage;
			m_fAdVoltagePair[1] = (float)nAdcAverage * (3.0/1023.0);	// 3.0V max, gain x5; 1023 max
			
			NRF_LOG_INFO("nAdcAverage = %d", m_nAdSamplesPair[1]);
			NRF_LOG_INFO("nAdcVoltageAverage = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_fAdVoltagePair[1]));
			
			compareAdcPairs((int32_t*)m_nAdSamplesPair);
			
			// Change to next state
			m_eState = STATE_IR_OFF;
		}

}

bool compareAdcPairs(int32_t* pnBuff)
{
	if( (pnBuff[1] - pnBuff[0]) > AD_OBJ_DET_TRESHOLD ) {
		m_unObjDetectionCounter++;
		NRF_LOG_INFO("m_unObjDetectionCounter: %u", m_unObjDetectionCounter);
	}
	else {
		m_unObjDetectionCounter = 0;	// Reset immediatly if one change was not detected
	}
	
	if(m_unObjDetectionCounter >= OBJ_DETECTION_CNT_MIN) {
		m_unObjDetectionCounter = OBJ_DETECTION_CNT_MIN; // Prevent possible overflow
		NRF_LOG_INFO("= = = = OBJECT DETECTED !");
		onboardLedWrite(BSP_LED_3, true);
		return true;
	}
	
	NRF_LOG_INFO("= = = = NO OBJECT");
	onboardLedWrite(BSP_LED_3, false);
	return false;
}

// 100us
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	
	// 10ms (100 Hz)
	if((m_ulMicros % 1000) == 0) {
		
		// 50Hz waveform IR diode
		if(m_eState == STATE_IR_OFF) {
			onboardLedWrite(BSP_LED_0, false);
			irLedWrite(false);
		}
		else if(m_eState == STATE_IR_ON) {
			onboardLedWrite(BSP_LED_0, true);
			irLedWrite(true);
		}

		// Trigger one sample
		nrf_drv_saadc_sample();
	}
	
	m_ulMicros+=100;
}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 0.1ms (100us) */
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 100);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&m_timer);

//    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
//                                                                                NRF_TIMER_CC_CHANNEL0);
//    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

//    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
//    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
//                                          timer_compare_event_addr,
//                                          saadc_sample_task_addr);
//    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

				onAdcDone(p_event->data.done.p_buffer);

//        int i;
//        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

//        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
//        {
//            NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
//        }
        m_adc_evt_counter++;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
	
		// Gain 1/5, internal ref 0.6V -> max deflection = 3.0V (1023)
		// AIN0 = P0.02
    nrf_saadc_channel_config_t channel_config = {
			.resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
			.resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
			.gain       = NRF_SAADC_GAIN1_5,                \
			.reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
			.acq_time   = NRF_SAADC_ACQTIME_10US,           \
			.mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
			.burst      = NRF_SAADC_BURST_DISABLED,         \
			.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0),       \
			.pin_n      = NRF_SAADC_INPUT_DISABLED  
		};
	
//        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);	// AIN0 = P0.02

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void leds_init(void)
{
	nrf_gpio_cfg_output(BSP_LED_0);
	nrf_gpio_cfg_output(BSP_LED_1);
	nrf_gpio_cfg_output(BSP_LED_2);
	nrf_gpio_cfg_output(BSP_LED_3);
	
	nrf_gpio_pin_set(BSP_LED_0);
	nrf_gpio_pin_set(BSP_LED_1);
	nrf_gpio_pin_set(BSP_LED_2);
	nrf_gpio_pin_set(BSP_LED_3);
}



void irDiodeInit(void)
{
	nrf_gpio_cfg_output(IR_LED_PIN);
	nrf_gpio_pin_clear(IR_LED_PIN);
}

void onboardLedWrite(uint32_t unBspLed, bool bOn) 
{
	
	APP_ERROR_CHECK( ((unBspLed >= BSP_LED_0) && (unBspLed <= BSP_LED_3)) ? NRF_SUCCESS : NRF_ERROR_INVALID_PARAM );
	
	// Iverted logic
	if(bOn)
		nrf_gpio_pin_clear(unBspLed);
	else
		nrf_gpio_pin_set(unBspLed);
}

void irLedWrite(bool bOn)
{
	if(bOn)
		nrf_gpio_pin_set(IR_LED_PIN);
	else
		nrf_gpio_pin_clear(IR_LED_PIN);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

    saadc_init();
    saadc_sampling_event_init();
//    saadc_sampling_event_enable(); // PPI
		irDiodeInit();
		leds_init();
    NRF_LOG_INFO("SAADC HAL simple example started.");

    while (1)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}


/** @} */
